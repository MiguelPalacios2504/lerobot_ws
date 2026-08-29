#include "lerobot_controller/feetech_bus.hpp"
#include <cmath>
#include <iostream>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

using namespace std::chrono_literals;

static std::vector<uint8_t> parse_ids(const std::string& ids_str) {
  std::vector<uint8_t> ids;
  size_t p = 0;
  while (p < ids_str.size()) {
    size_t q = ids_str.find(',', p);
    if (q == std::string::npos) q = ids_str.size();
    ids.push_back(static_cast<uint8_t>(std::stoi(ids_str.substr(p, q - p))));
    p = q + 1;
  }
  return ids;
}

int main(int argc, char** argv) {
  if (argc < 3) {
    std::cerr << "Uso: " << argv[0] << " <puerto> <id1,id2,...>\n";
    std::cerr << "Ejemplo: " << argv[0] << " /dev/ttyACM0 1,2,3,4,5,6\n";
    std::cerr << "Mueve cada joint a mano (torque off) o carga el brazo y observa load/corriente.\n";
    return 1;
  }

  const std::string port = argv[1];
  const auto ids = parse_ids(argv[2]);
  if (ids.empty()) {
    std::cerr << "Lista de IDs vacía.\n";
    return 1;
  }

  try {
    feetech::Bus bus;
    bus.open(port, 1000000);
    std::cout << "Puerto OK: " << port << " | IDs: " << argv[2] << "\n";
    std::cout << "id | load_signed | load_% | current_mA\n";

    while (true) {
      const auto states = bus.readAll(ids);
      for (size_t i = 0; i < ids.size(); ++i) {
        double current_ma = 0.0;
        try {
          current_ma = bus.readPresentCurrentMilliAmps(ids[i]);
        } catch (...) {
          current_ma = NAN;
        }
        const int load = states[i].load;
        const double load_pct = 100.0 * std::abs(load) / 1000.0;
        std::cout << static_cast<int>(ids[i]) << " | "
                  << load << " | "
                  << load_pct << "% | "
                  << current_ma << " mA\n";
      }
      std::cout << "---\n";
      std::this_thread::sleep_for(200ms);
    }
  } catch (const std::exception& e) {
    std::cerr << "Error: " << e.what() << "\n";
    return 2;
  }
}
