#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <errno.h>
#include <time.h>

// =========================
// CONFIGURACIÓN
// =========================
#define PORT "/dev/ttyACM0"
#define BAUDRATE B1000000
#define SERVO_COUNT 6

int servo_ids[SERVO_COUNT] = {1, 2, 3, 4, 5, 6};
int target_positions[SERVO_COUNT] = {2210, 794, 2391, 2939, 2130, 2048};

// =========================

int serial_fd;

unsigned char checksum_sum(unsigned char s) {
    return (~(s & 0xFF)) & 0xFF;
}

int open_serial_port() {
    struct termios options;
    
    serial_fd = open(PORT, O_RDWR | O_NOCTTY | O_SYNC);
    if (serial_fd < 0) {
        perror("Error abriendo puerto serie");
        return -1;
    }
    
    // Obtener configuración actual
    if (tcgetattr(serial_fd, &options) < 0) {
        perror("Error obteniendo atributos del puerto");
        close(serial_fd);
        return -1;
    }
    
    // Configurar velocidad
    cfsetispeed(&options, BAUDRATE);
    cfsetospeed(&options, BAUDRATE);
    
    // 8N1
    options.c_cflag &= ~PARENB;
    options.c_cflag &= ~CSTOPB;
    options.c_cflag &= ~CSIZE;
    options.c_cflag |= CS8;
    
    // Sin control de flujo
    options.c_cflag &= ~CRTSCTS;
    
    // Habilitar lectura
    options.c_cflag |= (CLOCAL | CREAD);
    
    // Configurar entrada raw
    options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
    
    // Configurar salida raw
    options.c_oflag &= ~OPOST;
    
    // Timeouts: retornar inmediatamente con lo que esté disponible
    options.c_cc[VMIN]  = 0;
    options.c_cc[VTIME] = 0;
    
    if (tcsetattr(serial_fd, TCSANOW, &options) < 0) {
        perror("Error configurando atributos del puerto");
        close(serial_fd);
        return -1;
    }
    
    // Limpiar buffers
    tcflush(serial_fd, TCIOFLUSH);
    usleep(100000); // 100ms
    
    return 0;
}

void send_packet(int sid, int inst, unsigned char* params, int param_length) {
    unsigned char packet[256];
    int length = param_length + 2;
    unsigned char body[256];
    int body_length = 0;
    
    // Construir body
    body[body_length++] = sid;
    body[body_length++] = length;
    body[body_length++] = inst;
    
    for (int i = 0; i < param_length; i++) {
        body[body_length++] = params[i];
    }
    
    // Calcular checksum
    int sum = 0;
    for (int i = 0; i < body_length; i++) {
        sum += body[i];
    }
    unsigned char chk = checksum_sum(sum);
    
    // Construir paquete completo
    int packet_length = 0;
    packet[packet_length++] = 0xFF;
    packet[packet_length++] = 0xFF;
    
    for (int i = 0; i < body_length; i++) {
        packet[packet_length++] = body[i];
    }
    
    packet[packet_length++] = chk;
    
    // Enviar paquete
    write(serial_fd, packet, packet_length);
    fsync(serial_fd);
}

void enable_torque(int sid) {
    unsigned char params[2] = {0x28, 0x01}; // ADDR_TORQUE_ENABLE, enable
    send_packet(sid, 0x03, params, 2);
    usleep(20000); // 20ms
}

void move_servo(int sid, int pos) {
    unsigned char params[8];
    
    // Limitar posición
    if (pos < 0) pos = 0;
    if (pos > 4095) pos = 4095;
    
    params[0] = 0x2A; // ADDR_GOAL_POS
    params[1] = pos & 0xFF; // Posición low
    params[2] = (pos >> 8) & 0xFF; // Posición high
    params[3] = 0x00; // Tiempo low (0 = máximo velocidad)
    params[4] = 0x00; // Tiempo high
    params[5] = 0x00; // Velocidad low
    params[6] = 0x00; // Velocidad high
    
    send_packet(sid, 0x03, params, 7);
    usleep(20000); // 20ms
}

int read_position(int sid) {
    unsigned char params[2] = {0x38, 0x02}; // ADDR_PRESENT_POS, leer 2 bytes
    unsigned char response[16];
    
    // Enviar comando de lectura
    send_packet(sid, 0x02, params, 2);
    
    // Leer respuesta
    usleep(10000); // 10ms
    int bytes_read = read(serial_fd, response, 16);
    
    if (bytes_read >= 8) {
        // Verificar header
        if (response[0] == 0xFF && response[1] == 0xFF) {
            // Verificar ID
            if (response[2] == sid) {
                int pos = response[5] | (response[6] << 8);
                if (pos <= 4095) {
                    return pos;
                }
            }
        }
    }
    
    return -1; // Error
}

int main() {
    printf("[INFO] Conectando a %s @ 1000000 baudios\n", PORT);
    
    if (open_serial_port() < 0) {
        return 1;
    }
    
    printf("[INFO] Conexión establecida ✅\n");
    usleep(500000); // 500ms
    
    // 1. Activar torque
    printf("\n[INFO] Activando torque...\n");
    for (int i = 0; i < SERVO_COUNT; i++) {
        enable_torque(servo_ids[i]);
        printf("  Servo %d: torque activado\n", servo_ids[i]);
    }
    printf("[OK] Torque activado ✅\n");
    
    usleep(500000); // 500ms
    
    // 2. Leer posiciones iniciales
    printf("\n[INFO] Leyendo posiciones iniciales...\n");
    for (int i = 0; i < SERVO_COUNT; i++) {
        int pos = read_position(servo_ids[i]);
        if (pos >= 0) {
            printf("  Servo %d: %d\n", servo_ids[i], pos);
        } else {
            printf("  Servo %d: NO RESPONDE\n", servo_ids[i]);
        }
    }
    
    // 3. Enviar posiciones objetivo
    printf("\n[INFO] Enviando posiciones objetivo...\n");
    for (int i = 0; i < SERVO_COUNT; i++) {
        printf("  Moviendo servo %d → %d\n", servo_ids[i], target_positions[i]);
        move_servo(servo_ids[i], target_positions[i]);
    }
    
    // 4. Esperar movimiento
    printf("\n[INFO] Esperando 2 segundos para movimiento...\n");
    sleep(2);
    
    // 5. Leer posiciones finales
    printf("\n[INFO] Leyendo posiciones finales...\n");
    printf("\n[RESULTADOS]:\n");
    for (int i = 0; i < SERVO_COUNT; i++) {
        int final_pos = read_position(servo_ids[i]);
        int target_pos = target_positions[i];
        
        if (final_pos >= 0) {
            int diff = abs(final_pos - target_pos);
            const char* status = (diff <= 50) ? "✅" : "⚠️ ";
            printf("  Servo %d: %d (objetivo: %d, diferencia: %d) %s\n", 
                   servo_ids[i], final_pos, target_pos, diff, status);
        } else {
            printf("  Servo %d: NO RESPONDE - objetivo: %d ❌\n", servo_ids[i], target_pos);
        }
    }
    
    printf("\n✅ Comando de movimiento completado\n");
    
    close(serial_fd);
    return 0;
}