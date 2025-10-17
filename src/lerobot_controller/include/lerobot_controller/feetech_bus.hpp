#pragma once
#include <cstdint>
#include <vector>
#include <string>
#include <stdexcept>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <sys/ioctl.h>
#include <errno.h>
#include <algorithm>

namespace feetech {

static constexpr uint8_t INST_READ  = 0x02;
static constexpr uint8_t INST_WRITE = 0x03;
static constexpr uint8_t HDR = 0xFF;

// Direcciones ST3215
static constexpr uint8_t ADDR_GOAL_POS      = 0x2A;
static constexpr uint8_t ADDR_MOVE_TIME     = 0x2C;
static constexpr uint8_t ADDR_PRESENT_POS   = 0x38;
static constexpr uint8_t ADDR_PRESENT_SPEED = 0x3A;
static constexpr uint8_t ADDR_PRESENT_LOAD  = 0x3C;

inline uint8_t chksum(uint8_t id, uint8_t length, uint8_t inst, const std::vector<uint8_t>& params) {
  uint32_t s = id + length + inst;
  for (auto b : params) s += b;
  return static_cast<uint8_t>(~(s & 0xFF));
}

class Bus {
public:
  Bus() : fd_(-1) {}
  ~Bus() { close(); }

  void open(const std::string& port, int baud = 1000000) {
    close();
    fd_ = ::open(port.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (fd_ < 0) throw std::runtime_error("No se pudo abrir puerto " + port);
    termios tio{};
    tcgetattr(fd_, &tio);
    cfmakeraw(&tio);
    tio.c_cflag |= CLOCAL | CREAD;
    tio.c_cflag &= ~CSTOPB;
    tio.c_cflag &= ~CRTSCTS;
    speed_t sp = B1000000;
    cfsetispeed(&tio, sp);
    cfsetospeed(&tio, sp);
    tcsetattr(fd_, TCSANOW, &tio);
    tcflush(fd_, TCIOFLUSH);
  }

  void close() { if (fd_ >= 0) { ::close(fd_); fd_ = -1; } }
  bool isOpen() const { return fd_ >= 0; }

  std::vector<uint8_t> readRegs(uint8_t id, uint8_t start, uint8_t len) {
    std::vector<uint8_t> params{start, len};
    sendPacket(id, INST_READ, params);
    auto rx = recvBytesBlocking(2 + 3 + len + 1, 5);
    size_t i = 0;
    while (i + 1 < rx.size() && !(rx[i]==HDR && rx[i+1]==HDR)) i++;
    if (i + 4 >= rx.size()) throw std::runtime_error("Respuesta sin cabecera");
    if (rx[i+2] != id) throw std::runtime_error("ID inesperado");
    if (rx[i+4] != 0x00) throw std::runtime_error("Status error=" + std::to_string(rx[i+4]));
    std::vector<uint8_t> data(len);
    for (uint8_t j=0;j<len;++j) data[j]=rx[i+5+j];
    return data;
  }

  // Lectura combinada de Posición, Velocidad y Carga
  struct ServoState {
    uint16_t pos{0};
    int16_t  vel{0};
    int16_t  load{0};
  };

  std::vector<ServoState> readAll(const std::vector<uint8_t>& ids) {
    std::vector<ServoState> states(ids.size());
    for (size_t i=0; i<ids.size(); ++i) {
      try {
        auto d = readRegs(ids[i], ADDR_PRESENT_POS, 6);
        states[i].pos  = static_cast<uint16_t>(d[0] | (d[1]<<8));
        states[i].vel  = static_cast<int16_t>(d[2] | (d[3]<<8));
        states[i].load = static_cast<int16_t>(d[4] | (d[5]<<8));
      } catch (...) {
        states[i] = ServoState{};
      }
    }
    return states;
  }

  void writeGoalPositionTime(uint8_t id, uint16_t pos, uint16_t time_ms, uint16_t speed = 0) {
    std::vector<uint8_t> data{
      static_cast<uint8_t>(pos & 0xFF),
      static_cast<uint8_t>((pos >> 8) & 0xFF),
      static_cast<uint8_t>(time_ms & 0xFF),
      static_cast<uint8_t>((time_ms >> 8) & 0xFF),
      static_cast<uint8_t>(speed & 0xFF),
      static_cast<uint8_t>((speed >> 8) & 0xFF)
    };
    writeRegs(id, ADDR_GOAL_POS, data);

    tcdrain(fd_);          // Espera a que todos los bytes se transmitan
    usleep(2000);          // (2 ms) deja que el transceptor libere la línea RS-485
    tcflush(fd_, TCIFLUSH); // Limpia posibles residuos en el buffer de entrada
  }

private:
  int fd_;
  void sendPacket(uint8_t id, uint8_t inst, const std::vector<uint8_t>& params) {
    uint8_t len = params.size() + 2;
    std::vector<uint8_t> tx{HDR,HDR,id,len,inst};
    tx.insert(tx.end(),params.begin(),params.end());
    tx.push_back(chksum(id,len,inst,params));
    writeAll(tx);
  }

  void writeRegs(uint8_t id, uint8_t addr, const std::vector<uint8_t>& data) {
    std::vector<uint8_t> p{addr};
    p.insert(p.end(),data.begin(),data.end());
    sendPacket(id, INST_WRITE, p);

    tcdrain(fd_);
    usleep(2000);
    tcflush(fd_, TCIFLUSH);
  }

  void writeAll(const std::vector<uint8_t>& buf) {
    size_t total=0;
    while (total<buf.size()) {
      ssize_t n=::write(fd_, buf.data()+total, buf.size()-total);
      if (n<0) {
        if (errno==EAGAIN){ usleep(100); continue; }
        throw std::runtime_error("write fallo");
      }
      total+=n;
    }
  }

  std::vector<uint8_t> recvBytesBlocking(size_t atleast,int timeout_ms){
    std::vector<uint8_t> out;
    const int step_us=1000;
    int waited=0;
    while(waited<=timeout_ms){
      uint8_t tmp[256];
      ssize_t n=::read(fd_,tmp,sizeof(tmp));
      if(n>0){ out.insert(out.end(),tmp,tmp+n); if(out.size()>=atleast) break; }
      usleep(step_us);
      waited++;
    }
    return out;
  }
};

} // namespace feetech
