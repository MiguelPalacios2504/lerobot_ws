#pragma once

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/state.hpp>
#include <hardware_interface/system_interface.hpp>
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <string>
#include <vector>
#include "lerobot_controller/feetech_bus.hpp"
#include <deque>

namespace lerobot_controller {

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

class LerobotInterface : public hardware_interface::SystemInterface {
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(LerobotInterface)

  LerobotInterface();
  ~LerobotInterface() override;

  CallbackReturn on_init(const hardware_interface::HardwareInfo & info) override;
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;
  CallbackReturn on_activate(const rclcpp_lifecycle::State &) override;
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State &) override;
  hardware_interface::return_type read(const rclcpp::Time &, const rclcpp::Duration &) override;
  hardware_interface::return_type write(const rclcpp::Time &, const rclcpp::Duration &) override;

private:
  feetech::Bus bus_;
  std::string port_ = "/dev/ttyACM0";
  int baud_ = 1000000;
  std::vector<uint8_t> ids_ {1,2,3,4,5,6};
  uint16_t move_time_ms_ = 200;

  double scale_ = 4096.0 / (2.0 * M_PI);
  std::vector<int> offsets_raw_;
  std::vector<int> signs_;

  // Buffers ROS control
  std::vector<double> position_commands_;
  std::vector<double> position_states_;
  std::vector<double> velocity_states_;
  std::vector<double> effort_states_;
  std::vector<double> prev_position_commands_;

  std::vector<std::deque<double>> vel_history_;
  std::vector<double> vel_filtered_;
  size_t vel_filter_window_ = 5;
  double vel_deadband_ = 0.05;

  std::vector<std::deque<double>> pos_history_;
  std::vector<double> pos_filtered_;
  const size_t pos_filter_window_ = 15;
  const double pos_deadband_ = 0.0005;

  // utilidades
  uint16_t rad_to_raw(int i, double rad) const;
  double   raw_to_rad(int i, uint16_t raw) const;
};

} // namespace lerobot_controller
