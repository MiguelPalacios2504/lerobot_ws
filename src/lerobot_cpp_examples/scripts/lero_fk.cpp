#include <stdio.h>
#include <math.h>
#include <string.h>
#include <stdbool.h>

// ---------- Parámetros DH ----------
const double L0  = 54.2;
const double L1  = 30.5;
const double L2  = 116.1;
const double L3  = 134.5;
const double L45 = 160.0;

// ---------- Funciones de utilidad ----------
inline double rad(double deg) { return deg * M_PI / 180.0; }
inline double deg(double rad) { return rad * 180.0 / M_PI; }

void mat4_mul(const double A[4][4], const double B[4][4], double C[4][4]) {
    for (int i=0; i<4; ++i)
        for (int j=0; j<4; ++j) {
            double s=0;
            for (int k=0; k<4; ++k) s += A[i][k]*B[k][j];
            C[i][j]=s;
        }
}

void T_dh(double theta, double d, double a, double alpha, double T[4][4]) {
    double ct = cos(theta), st = sin(theta);
    double ca = cos(alpha), sa = sin(alpha);
    T[0][0] =  ct;  T[0][1] = -st*ca;  T[0][2] =  st*sa;  T[0][3] = a*ct;
    T[1][0] =  st;  T[1][1] =  ct*ca;  T[1][2] = -ct*sa;  T[1][3] = a*st;
    T[2][0] =   0;  T[2][1] =     sa;  T[2][2] =     ca;  T[2][3] =    d;
    T[3][0] =   0;  T[3][1] =      0;  T[3][2] =      0;  T[3][3] =    1;
}

void fk_5dof(const double q[5], double T05[4][4]) {
    double T[4][4] = {{1,0,0,0},{0,1,0,0},{0,0,1,0},{0,0,0,1}}, A[4][4];
    T_dh(q[0], L0, L1, M_PI/2, A);         mat4_mul(T, A, T);
    T_dh(q[1], 0, L2, 0, A);               mat4_mul(T, A, T);
    T_dh(q[2], 0, L3, 0, A);               mat4_mul(T, A, T);
    T_dh(q[3]-M_PI/2, 0, 0, -M_PI/2, A);   mat4_mul(T, A, T);
    T_dh(q[4], L45, 0, 0, A);              mat4_mul(T, A, T);
    memcpy(T05, T, sizeof(double)*16);
}

void print_T(const char* label, double T[4][4]) {
    printf("\n%s:\n", label);
    for (int i=0;i<4;i++) {
        for (int j=0;j<4;j++) printf("%10.3f ", T[i][j]);
        printf("\n");
    }
    printf("Posición (x,y,z): %.2f, %.2f, %.2f mm\n", T[0][3], T[1][3], T[2][3]);
}

// ---------- Prueba de dos configuraciones ----------
int main() {
    double q1[5] = { rad(90), rad(90), rad(0), rad(0), rad(0) };
    double q2[5] = { rad(90), rad(90), rad(-90), rad(0), rad(0) };

    double T1[4][4], T2[4][4];
    fk_5dof(q1, T1);
    fk_5dof(q2, T2);

    printf("=== Verificación FK Piper 5DOF ===\n");
    print_T("Config A [90,90,0,0,0]", T1);
    print_T("Config B [90,90,-90,0,0]", T2);

    // Estas matrices (posiciones) deben coincidir con tu Python
    // Puedes usarlas como "objetivos" para la IK más adelante
    return 0;
}
