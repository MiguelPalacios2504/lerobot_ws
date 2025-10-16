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
    if (info_.hardware_parameters.count("move_time_ms")) move_time_ms_ = static_cast<uint16_t>(std::stoi(info_.hardware_parameters.at("move_time_ms")));
    if (info_.hardware_parameters.count("ids")) {
      ids_.clear();
      std::string s = info_.hardware_parameters.at("ids");
      size_t p=0;
      while (p < s.size()) {
        size_t q = s.find(',', p);
        if (q == std::string::npos) q = s.size();
        int v = std::stoi(s.substr(p, q-p));
        ids_.push_back(static_cast<uint8_t>(v));
        p = q + 1;
      }
    }
  } catch (...) {
    RCLCPP_ERROR(rclcpp::get_logger("LerobotInterface"), "Error leyendo parametros de hardware");
    return CallbackReturn::FAILURE;
  }

  const size_t n = info_.joints.size();
  position_commands_.assign(n, 0.0);
  position_states_.assign(n, 0.0);
  prev_position_commands_.assign(n, 1e9);

  offsets_raw_ = {
    2179,
    3594,
    345,
    2354,
    2165,
    2275 
  };

  // Sentidos según calibración física
  signs_ = {+1, -1, -1, -1, -1, +1};
  scale_ = 4096.0 / (2.0 * M_PI);

  if (ids_.size() != n) {
    RCLCPP_WARN(rclcpp::get_logger("LerobotInterface"),
      "El numero de IDs (%zu) no coincide con joints (%zu). Se usara min.", ids_.size(), n);
    ids_.resize(n);
    for (size_t i=0;i<n;i++) if (ids_[i]==0) ids_[i]=static_cast<uint8_t>(i+1);
  }

  return CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> LerobotInterface::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> out;
  out.reserve(info_.joints.size());
  for (size_t i=0;i<info_.joints.size();++i) {
    out.emplace_back(info_.joints[i].name, hardware_interface::HW_IF_POSITION, &position_states_[i]);
  }
  return out;
}

std::vector<hardware_interface::CommandInterface> LerobotInterface::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> out;
  out.reserve(info_.joints.size());
  for (size_t i=0;i<info_.joints.size();++i) {
    out.emplace_back(info_.joints[i].name, hardware_interface::HW_IF_POSITION, &position_commands_[i]);
  }
  return out;
}

CallbackReturn LerobotInterface::on_activate(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(rclcpp::get_logger("LerobotInterface"), "Abriendo bus Feetech en %s @ %d", port_.c_str(), baud_);
  try {
    bus_.open(port_, baud_);
  } catch (const std::exception& e) {
    RCLCPP_FATAL(rclcpp::get_logger("LerobotInterface"), "No se pudo abrir el puerto: %s", e.what());
    return CallbackReturn::FAILURE;
  }
  // Estado inicial = lectura real
  try {
    for (size_t i=0;i<info_.joints.size();++i) {
      uint16_t raw = bus_.readPresentPosition(ids_[i]);
      position_states_[i] = raw_to_rad(i, raw);
      position_commands_[i] = position_states_[i];
    }
    prev_position_commands_ = position_commands_;
  } catch (const std::exception& e) {
    RCLCPP_WARN(rclcpp::get_logger("LerobotInterface"), "Lectura inicial fallo: %s", e.what());
  }

  RCLCPP_INFO(rclcpp::get_logger("LerobotInterface"), "Hardware listo (lectura/escritura).");
  return CallbackReturn::SUCCESS;
}

CallbackReturn LerobotInterface::on_deactivate(const rclcpp_lifecycle::State &)
{
  bus_.close();
  RCLCPP_INFO(rclcpp::get_logger("LerobotInterface"), "Bus cerrado.");
  return CallbackReturn::SUCCESS;
}

hardware_interface::return_type
LerobotInterface::read(const rclcpp::Time &, const rclcpp::Duration &)
{
  try {
    // Lectura sincronizada tipo SYNC_READ
    auto positions = bus_.syncReadPositions(ids_);
    if (positions.size() == info_.joints.size()) {
      for (size_t i = 0; i < positions.size(); ++i)
        position_states_[i] = raw_to_rad(i, positions[i]);
    } else {
      RCLCPP_WARN(rclcpp::get_logger("LerobotInterface"),
                  "Lectura incompleta (%zu/%zu servos)", positions.size(), ids_.size());
    }
  } catch (const std::exception& e) {
    RCLCPP_ERROR(rclcpp::get_logger("LerobotInterface"),
                 "Error leyendo servos: %s", e.what());
    return hardware_interface::return_type::ERROR;
  }
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type
LerobotInterface::write(const rclcpp::Time &, const rclcpp::Duration &)
{
  bool changed = false;
  for (size_t i = 0; i < position_commands_.size(); ++i)
    if (position_commands_[i] != prev_position_commands_[i]) { changed = true; break; }
  if (!changed) return hardware_interface::return_type::OK;

  try {
    for (size_t i = 0; i < info_.joints.size(); ++i) {
      uint16_t raw = rad_to_raw(i, position_commands_[i]);
      bus_.writeGoalPositionTime(ids_[i], raw, move_time_ms_, 0);
    }
    prev_position_commands_ = position_commands_;
  } catch (const std::exception& e) {
    RCLCPP_ERROR(rclcpp::get_logger("LerobotInterface"),
                 "Error escribiendo servos: %s", e.what());
    return hardware_interface::return_type::ERROR;
  }
  return hardware_interface::return_type::OK;
}

uint16_t LerobotInterface::rad_to_raw(int i, double rad) const {
  // raw = offset + sign * scale * rad
  double val = static_cast<double>(offsets_raw_[i]) + static_cast<double>(signs_[i]) * (scale_ * rad);
  if (val < 0.0) val = 0.0;
  if (val > 4095.0) val = 4095.0;
  return static_cast<uint16_t>(std::lround(val));
}

double LerobotInterface::raw_to_rad(int i, uint16_t raw) const {
  int centered = static_cast<int>(raw) - offsets_raw_[i];
  double rad = static_cast<double>(centered) / scale_;
  return static_cast<double>(signs_[i]) * rad;
}

}

PLUGINLIB_EXPORT_CLASS(lerobot_controller::LerobotInterface, hardware_interface::SystemInterface)
