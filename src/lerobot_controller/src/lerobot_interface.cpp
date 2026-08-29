#include "lerobot_controller/lerobot_interface.hpp"
#include <cmath>

namespace lerobot_controller {

LerobotInterface::LerobotInterface() {}
LerobotInterface::~LerobotInterface() { bus_.close(); }

CallbackReturn LerobotInterface::on_init(const hardware_interface::HardwareInfo & info)
{
  auto ret = hardware_interface::SystemInterface::on_init(info);
  if (ret != CallbackReturn::SUCCESS) return ret;

  try {
    if (info_.hardware_parameters.count("port")) port_ = info_.hardware_parameters.at("port");
    if (info_.hardware_parameters.count("baud")) baud_ = std::stoi(info_.hardware_parameters.at("baud"));
    if (info_.hardware_parameters.count("move_time_ms"))
      move_time_ms_ = static_cast<uint16_t>(std::stoi(info_.hardware_parameters.at("move_time_ms")));
    if (info_.hardware_parameters.count("passive_mode"))
      passive_mode_ = (info_.hardware_parameters.at("passive_mode") == "true");
    if (info_.hardware_parameters.count("command_deadband"))
      command_deadband_ = std::stod(info_.hardware_parameters.at("command_deadband"));
    if (info_.hardware_parameters.count("max_effort_nm"))
      max_effort_nm_ = std::stod(info_.hardware_parameters.at("max_effort_nm"));
  } catch (...) {
    RCLCPP_ERROR(rclcpp::get_logger("LerobotInterface"), "Error leyendo parámetros de hardware");
    return CallbackReturn::FAILURE;
  }

  const size_t n = info_.joints.size();
  position_commands_.assign(n, 0.0);
  position_states_.assign(n, 0.0);
  velocity_states_.assign(n, 0.0);
  effort_states_.assign(n, 0.0);
  prev_position_commands_.assign(n, 1e9);
  vel_history_.resize(n);
  vel_filtered_.assign(n, 0.0);
  pos_history_.resize(n);
  pos_filtered_.assign(n, 0.0);


  // Calibración por servo (ticks Feetech @ pose 0°). ID1 sin cambios (2026-08).
  offsets_raw_ = {
    2179,  // ID1
    936,   // ID2 @ 0°
    3061,  // ID3 @ 0°
    2067,  // ID4 @ 0°
    2055,  // ID5 @ 0°
    2160,  // ID6 @ 0°
  };
  signs_ = {-1, -1, -1, -1, -1, -1};
  scale_ = 4096.0 / (2.0 * M_PI);

  return CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> LerobotInterface::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> out;
  for (size_t i=0; i<info_.joints.size(); ++i) {
  out.emplace_back(info_.joints[i].name, hardware_interface::HW_IF_POSITION, &position_states_[i]);
  out.emplace_back(info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &velocity_states_[i]);
  out.emplace_back(info_.joints[i].name, hardware_interface::HW_IF_EFFORT,   &effort_states_[i]);
  }
  return out;
}

std::vector<hardware_interface::CommandInterface> LerobotInterface::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> out;
  for (size_t i = 0; i < info_.joints.size(); ++i)
    out.emplace_back(info_.joints[i].name, hardware_interface::HW_IF_POSITION, &position_commands_[i]);
  return out;
}

CallbackReturn LerobotInterface::on_activate(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(rclcpp::get_logger("LerobotInterface"),
              "Abriendo bus Feetech en %s @ %d", port_.c_str(), baud_);
  try {
    bus_.open(port_, baud_);
  } catch (const std::exception& e) {
    RCLCPP_FATAL(rclcpp::get_logger("LerobotInterface"),
                 "No se pudo abrir el puerto: %s", e.what());
    return CallbackReturn::FAILURE;
  }

  try {
    auto states = bus_.readAll(ids_);
    for (size_t i = 0; i < info_.joints.size(); ++i) {
      if (i < states.size()) {
        const double internal = raw_to_rad(i, states[i].pos);
        pos_filtered_[i] = internal;
        position_states_[i] = to_urdf_angle(static_cast<int>(i), internal);
        position_commands_[i] = position_states_[i];
      }
    }
    prev_position_commands_ = position_commands_;

    if (passive_mode_) {
      for (uint8_t id : ids_) {
        bus_.setTorqueEnable(id, false);
      }
      RCLCPP_INFO(rclcpp::get_logger("LerobotInterface"),
                  "Modo pasivo: torque desactivado (brazo líder).");
    }
  } catch (const std::exception &e) {
    RCLCPP_WARN(rclcpp::get_logger("LerobotInterface"),
                "Lectura inicial falló: %s", e.what());
  }


  RCLCPP_INFO(rclcpp::get_logger("LerobotInterface"),
              "Hardware listo (lectura/escritura).");
  return CallbackReturn::SUCCESS;
}

CallbackReturn LerobotInterface::on_deactivate(const rclcpp_lifecycle::State &)
{
  if (passive_mode_) {
    try {
      for (uint8_t id : ids_) {
        bus_.setTorqueEnable(id, true);
      }
    } catch (const std::exception &e) {
      RCLCPP_WARN(rclcpp::get_logger("LerobotInterface"),
                  "No se pudo reactivar torque: %s", e.what());
    }
  }
  bus_.close();
  RCLCPP_INFO(rclcpp::get_logger("LerobotInterface"), "Bus cerrado.");
  return CallbackReturn::SUCCESS;
}

hardware_interface::return_type LerobotInterface::read(const rclcpp::Time &, const rclcpp::Duration &)
{
  try {
    auto states = bus_.readAll(ids_);

    // 🔧 Conversión a unidades físicas
    const double TICKS_TO_RAD = (2.0 * M_PI / 4096.0);
    const double SPEED_SCALE  = TICKS_TO_RAD * 10.0;   // ticks/0.1s → rad/s
    constexpr double LOAD_FULL_SCALE = 1000.0;       // ±1000 = 100% torque ST3215

    for (size_t i = 0; i < states.size(); ++i) {
      // --- POSICIÓN ---
      double pos_value = raw_to_rad(i, states[i].pos);

      // Filtro de media móvil
      auto &p_hist = pos_history_[i];
      p_hist.push_back(pos_value);
      if (p_hist.size() > pos_filter_window_)
        p_hist.pop_front();

      double p_avg = 0.0;
      for (double v : p_hist) p_avg += v;
      p_avg /= p_hist.size();

      // Deadband para pequeños ruidos
      if (std::fabs(p_avg - pos_filtered_[i]) < pos_deadband_)
        p_avg = pos_filtered_[i];

      pos_filtered_[i] = p_avg;
      position_states_[i] = to_urdf_angle(static_cast<int>(i), pos_filtered_[i]);


      int16_t raw_vel = std::clamp<int16_t>(states[i].vel, 0, 2047);

      // 0–1023: CW (+), 1024–2047: CCW (−)
      int16_t signed_vel;
      if (raw_vel <= 1023)
        signed_vel = raw_vel;
      else
        signed_vel = -static_cast<int16_t>(raw_vel - 1024);

      double vel_value = signed_vel * SPEED_SCALE;



      // Filtro de media móvil (N=5 por defecto)
      auto &hist = vel_history_[i];
      hist.push_back(vel_value);
      if (hist.size() > vel_filter_window_) hist.pop_front();
      double avg = 0.0;
      for (double v : hist) avg += v;
      double filtered = avg / hist.size();

      // Zona muerta (deadband) para eliminar ruido residual
      if (std::fabs(filtered) < vel_deadband_) filtered = 0.0;

      velocity_states_[i] = to_urdf_angle(static_cast<int>(i), filtered);

      // --- TORQUE (Present Load → Nm aproximado) ---
      const double load_norm = static_cast<double>(states[i].load) / LOAD_FULL_SCALE;
      effort_states_[i] = load_norm * max_effort_nm_;
    }

  } catch (const std::exception &e) {
    RCLCPP_ERROR(rclcpp::get_logger("LerobotInterface"),
                 "Error leyendo servos: %s", e.what());
    return hardware_interface::return_type::ERROR;
  }

  return hardware_interface::return_type::OK;
}



hardware_interface::return_type LerobotInterface::write(const rclcpp::Time &, const rclcpp::Duration &)
{
  if (passive_mode_) {
    return hardware_interface::return_type::OK;
  }

  try {
    for (size_t i = 0; i < info_.joints.size(); ++i) {
      if (std::fabs(position_commands_[i] - prev_position_commands_[i]) <= command_deadband_) {
        continue;
      }
      uint16_t raw = rad_to_raw(i, from_urdf_angle(static_cast<int>(i), position_commands_[i]));
      bus_.writeGoalPositionTime(ids_[i], raw, move_time_ms_, 0);
      prev_position_commands_[i] = position_commands_[i];
    }
  } catch (const std::exception& e) {
    RCLCPP_ERROR(rclcpp::get_logger("LerobotInterface"),
                 "Error escribiendo servos: %s", e.what());
    return hardware_interface::return_type::ERROR;
  }

  return hardware_interface::return_type::OK;
}

uint16_t LerobotInterface::rad_to_raw(int i, double rad) const {
  double val = static_cast<double>(offsets_raw_[i]) +
               static_cast<double>(signs_[i]) * (scale_ * rad);
  val = std::clamp(val, 0.0, 4095.0);
  return static_cast<uint16_t>(std::lround(val));
}

double LerobotInterface::raw_to_rad(int i, uint16_t raw) const {
  int centered = static_cast<int>(raw) - offsets_raw_[i];
  return static_cast<double>(signs_[i]) * (centered / scale_);
}

double LerobotInterface::to_urdf_angle(int i, double rad) const {
  if (i >= 1 && i <= 4) return -rad;
  return rad;
}

double LerobotInterface::from_urdf_angle(int i, double rad) const {
  if (i >= 1 && i <= 4) return -rad;
  return rad;
}

} // namespace lerobot_controller

PLUGINLIB_EXPORT_CLASS(lerobot_controller::LerobotInterface, hardware_interface::SystemInterface)
