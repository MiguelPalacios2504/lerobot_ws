#pragma once
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/state.hpp>
#include <hardware_interface/system_interface.hpp>
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <string>
#include <vector>
#include "lerobot_controller/feetech_bus.hpp"

namespace lerobot_controller {

  using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

  class LerobotInterface : public hardware_interface::SystemInterface {
  public:
    RCLCPP_SHARED_PTR_DEFINITIONS(LerobotInterface)

    LerobotInterface();
    ~LerobotInterface() override;

    // ROS 2 Control
    CallbackReturn on_init(const hardware_interface::HardwareInfo & info) override;
    std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
    std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;
    CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;
    CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;
    hardware_interface::return_type read(const rclcpp::Time & time, const rclcpp::Duration & period) override;
    hardware_interface::return_type write(const rclcpp::Time & time, const rclcpp::Duration & period) override;

  private:
    // It acctually depends on the servo u use.
    feetech::Bus bus_;
    std::string port_ = "/dev/ttyACM0";
    int baud_ = 1000000;

    // You cand modifie in case you use more servos
    std::vector<uint8_t> ids_ {1,2,3,4,5,6};

    double scale_ = 4096.0 / (2.0 * M_PI);
    std::vector<int>    offsets_raw_; // INICIAL VALUES
    std::vector<int>    signs_;

    uint16_t move_time_ms_ = 200;

    // Buffers ROS control
    std::vector<double> position_commands_;
    std::vector<double> position_states_;
    std::vector<double> prev_position_commands_;

    // utilidades
    uint16_t rad_to_raw(int i, double rad) const;
    double   raw_to_rad(int i, uint16_t raw) const;
  };

};
