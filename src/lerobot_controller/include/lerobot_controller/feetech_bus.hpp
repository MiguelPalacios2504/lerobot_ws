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

namespace feetech {

// Registros básicos SC/STS (compatibles ST3215)
static constexpr uint8_t INST_PING   = 0x01;
static constexpr uint8_t INST_READ   = 0x02;
static constexpr uint8_t INST_WRITE  = 0x03;

static constexpr uint8_t HDR = 0xFF;

// Direcciones de interés
static constexpr uint8_t ADDR_GOAL_POS     = 0x2A; // 2 bytes (L,H)
static constexpr uint8_t ADDR_MOVE_TIME    = 0x2C; // 2 bytes (L,H)
static constexpr uint8_t ADDR_PRESENT_POS  = 0x3A; // 2 bytes (L,H)

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
    if (fd_ < 0) throw std::runtime_error("feetech::Bus: no se pudo abrir " + port + " errno=" + std::to_string(errno));
    termios tio{};
    if (tcgetattr(fd_, &tio) != 0) throw std::runtime_error("tcgetattr fallo");
    cfmakeraw(&tio);
    tio.c_cflag |= CLOCAL | CREAD;
    tio.c_cflag &= ~CSTOPB;
    tio.c_cflag &= ~CRTSCTS;

    speed_t sp = B1000000; // 1 Mbps típico de ST3215
    // Si requieres otro baudrate, aquí puedes mapearlo.
    cfsetispeed(&tio, sp);
    cfsetospeed(&tio, sp);
    if (tcsetattr(fd_, TCSANOW, &tio) != 0) throw std::runtime_error("tcsetattr fallo");
    tcflush(fd_, TCIOFLUSH);
  }

  void close() {
    if (fd_ >= 0) { ::close(fd_); fd_ = -1; }
  }

  bool isOpen() const { return fd_ >= 0; }

  // WRITE registers
  void writeRegs(uint8_t id, uint8_t start_addr, const std::vector<uint8_t>& data) {
    std::vector<uint8_t> params;
    params.reserve(1 + data.size());
    params.push_back(start_addr);
    params.insert(params.end(), data.begin(), data.end());
    sendPacket(id, INST_WRITE, params);
    // Muchos drivers no envían status si no lo pides; no esperamos respuesta.
  }

  // READ registers
  std::vector<uint8_t> readRegs(uint8_t id, uint8_t start_addr, uint8_t len) {
    std::vector<uint8_t> params{start_addr, len};
    sendPacket(id, INST_READ, params);
    // Respuesta: 0xFF 0xFF ID LENGTH 0x00 <data...> CHKSUM
    // LENGTH = data_len + 2 (status + checksum)
    auto rx = recvBytesBlocking(2 + 3 + len + 1, 5); // margen
    // Buscar cabecera
    size_t idx = 0;
    while (idx + 2 <= rx.size() && !(rx[idx] == HDR && rx[idx+1] == HDR)) idx++;
    if (idx + 4 >= rx.size()) throw std::runtime_error("Respuesta sin cabecera");
    if (rx[idx+2] != id) throw std::runtime_error("ID inesperado en respuesta");
    uint8_t length = rx[idx+3];
    if (idx + 2 + 2 + length > rx.size()) throw std::runtime_error("Longitud de respuesta inconsistente");
    uint8_t status = rx[idx+4];
    if (status != 0x00) throw std::runtime_error("Status error=" + std::to_string(status));
    std::vector<uint8_t> data(len);
    for (uint8_t i=0; i<len; ++i) data[i] = rx[idx+5+i];
    // (Opcional) verificar checksum
    return data;
  }

  std::vector<uint16_t> syncReadPositions(const std::vector<uint8_t>& ids)
  {
    if (ids.empty()) return {};

    const uint8_t INST_SYNC_READ = 0x82;
    const uint8_t ADDR_START_READ = 0x38;  // posición actual
    const uint8_t READ_LENGTH = 2;         // 2 bytes (pos)

    std::vector<uint8_t> params;
    params.reserve(3 + ids.size());
    params.push_back(ADDR_START_READ);
    params.push_back(READ_LENGTH);
    params.push_back(ids.size());
    params.insert(params.end(), ids.begin(), ids.end());

    sendPacket(0xFE, INST_SYNC_READ, params);

    // Espera datos de todos los servos
    const int expected = (READ_LENGTH + 6) * ids.size();
    auto raw = recvBytesBlocking(expected * 2, 5); // 5 ms total máximo

    std::vector<uint16_t> positions(ids.size(), 0);
    size_t i = 0;
    while (i + READ_LENGTH + 5 < raw.size()) {
      if (raw[i] == 0xFF && raw[i + 1] == 0xFF) {
        uint8_t sid = raw[i + 2];
        auto it = std::find(ids.begin(), ids.end(), sid);
        if (it != ids.end()) {
          size_t idx = std::distance(ids.begin(), it);
          uint8_t lo = raw[i + 5];
          uint8_t hi = raw[i + 6];
          positions[idx] = static_cast<uint16_t>(lo | (hi << 8));
          i += READ_LENGTH + 6;
          continue;
        }
      }
      ++i;
    }
    return positions;
  }

  // Conveniencias
  uint16_t readPresentPosition(uint8_t id) {
    auto d = readRegs(id, ADDR_PRESENT_POS, 2);
    return static_cast<uint16_t>(d[0] | (static_cast<uint16_t>(d[1]) << 8));
  }

  void writeGoalPositionTime(uint8_t id, uint16_t pos, uint16_t time_ms, uint16_t speed = 0) {
    std::vector<uint8_t> data{
      static_cast<uint8_t>(pos & 0xFF), static_cast<uint8_t>((pos >> 8) & 0xFF),
      static_cast<uint8_t>(time_ms & 0xFF), static_cast<uint8_t>((time_ms >> 8) & 0xFF),
      static_cast<uint8_t>(speed & 0xFF), static_cast<uint8_t>((speed >> 8) & 0xFF)
    };
    writeRegs(id, ADDR_GOAL_POS, data);
  }

private:
  int fd_;

  void sendPacket(uint8_t id, uint8_t inst, const std::vector<uint8_t>& params) {
    uint8_t length = static_cast<uint8_t>(params.size() + 2);
    std::vector<uint8_t> tx;
    tx.reserve(2 + 3 + params.size() + 1);
    tx.push_back(HDR); tx.push_back(HDR);
    tx.push_back(id);
    tx.push_back(length);
    tx.push_back(inst);
    tx.insert(tx.end(), params.begin(), params.end());
    tx.push_back(chksum(id, length, inst, params));
    writeAll(tx);
  }

  void writeAll(const std::vector<uint8_t>& buf) {
    size_t total = 0;
    while (total < buf.size()) {
      ssize_t n = ::write(fd_, buf.data() + total, buf.size() - total);
      if (n < 0) {
        if (errno == EAGAIN) { usleep(100); continue; }
        throw std::runtime_error("write fallo errno=" + std::to_string(errno));
      }
      total += static_cast<size_t>(n);
    }
  }

  // lectura simple con timeout (ms)
  std::vector<uint8_t> recvBytesBlocking(size_t atleast, int timeout_ms) {
    std::vector<uint8_t> out;
    out.reserve(atleast + 8);
    const int step_us = 1000; // 1 ms
    int waited = 0;
    while (waited <= timeout_ms) {
      uint8_t tmp[256];
      ssize_t n = ::read(fd_, tmp, sizeof(tmp));
      if (n > 0) {
        out.insert(out.end(), tmp, tmp + n);
        if (out.size() >= atleast) break;
      } else if (n < 0 && errno != EAGAIN) {
        throw std::runtime_error("read fallo errno=" + std::to_string(errno));
      }
      usleep(step_us);
      waited += 1;
    }
    return out;
  }
};

} // namespace feetech
