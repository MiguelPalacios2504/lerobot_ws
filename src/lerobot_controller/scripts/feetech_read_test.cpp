#include "lerobot_controller/feetech_bus.hpp"
#include <iostream>
#include <vector>
#include <cmath>
#include <thread>
#include <chrono>

using namespace std::chrono_literals;

int main(int argc, char **argv)
{
    if (argc < 3)
    {
        std::cerr << "Uso: " << argv[0] << " <puerto> <id1,id2,id3,...>\n";
        std::cerr << "Ejemplo: sudo ./feetech_read_loop /dev/ttyACM0 1,2,3,4,5,6\n";
        return 1;
    }

    std::string port = argv[1];
    std::vector<uint8_t> ids;
    std::string ids_str = argv[2];
    size_t p = 0;
    while (p < ids_str.size())
    {
        size_t q = ids_str.find(',', p);
        if (q == std::string::npos)
            q = ids_str.size();
        ids.push_back(static_cast<uint8_t>(std::stoi(ids_str.substr(p, q - p))));
        p = q + 1;
    }

    if (ids.empty())
    {
        std::cerr << "❌ Lista de IDs vacía.\n";
        return 1;
    }

    try
    {
        feetech::Bus bus;
        bus.open(port, 1000000);
        std::cout << "✅ Puerto abierto correctamente: " << port << "\n";

        const double ticks_to_rad = (2.0 * M_PI) / 4096.0;   // 1 vuelta = 4096 ticks
        const double vel_factor   = ticks_to_rad * 10.0;     // ≈ conversión a rad/s
        const double torque_factor = 1.0 / 1023.0;           // normalizado (-1..1)

        while (true)
        {
            std::vector<double> positions;
            std::vector<double> velocities;
            std::vector<double> efforts;

            for (uint8_t id : ids)
            {
                try
                {
                    uint16_t pos_raw = bus.readPresentPosition(id);
                    int16_t vel_raw  = bus.readPresentVelocitySigned(id);
                    int16_t cur_raw  = bus.readPresentCurrentSigned(id);

                    double pos = pos_raw * ticks_to_rad;
                    double vel = vel_raw * vel_factor;
                    double eff = cur_raw * torque_factor;

                    positions.push_back(pos);
                    velocities.push_back(vel);
                    efforts.push_back(eff);
                }
                catch (const std::exception &e)
                {
                    std::cerr << "⚠️ Servo ID " << (int)id << " error: " << e.what() << "\n";
                    positions.push_back(NAN);
                    velocities.push_back(NAN);
                    efforts.push_back(NAN);
                }
            }

            // Formato tipo JointState (una sola línea)
            std::cout << "pos:[";
            for (size_t i = 0; i < positions.size(); ++i)
            {
                std::cout << positions[i];
                if (i + 1 < positions.size()) std::cout << ",";
            }
            std::cout << "] vel:[";
            for (size_t i = 0; i < velocities.size(); ++i)
            {
                std::cout << velocities[i];
                if (i + 1 < velocities.size()) std::cout << ",";
            }
            std::cout << "] eff:[";
            for (size_t i = 0; i < efforts.size(); ++i)
            {
                std::cout << efforts[i];
                if (i + 1 < efforts.size()) std::cout << ",";
            }
            std::cout << "]" << std::endl;

            std::this_thread::sleep_for(20ms); // ~50 Hz
        }
    }
    catch (const std::exception &e)
    {
        std::cerr << "❌ Error al iniciar: " << e.what() << "\n";
        return 2;
    }

    return 0;
}
