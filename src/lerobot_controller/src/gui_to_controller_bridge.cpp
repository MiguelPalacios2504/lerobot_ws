#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

class GUIToControllerBridge : public rclcpp::Node
{
public:
  GUIToControllerBridge()
  : Node("gui_to_controller_bridge")
  {
    using std::placeholders::_1;

    sub_joint_states_ = this->create_subscription<sensor_msgs::msg::JointState>(
      "/joint_states", 10, std::bind(&GUIToControllerBridge::jointCallback, this, _1));

    pub_arm_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
      "/arm_controller/commands", 10);

    pub_gripper_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
      "/gripper_controller/commands", 10);

    RCLCPP_INFO(this->get_logger(), "✅ Bridge iniciado: joint_states → arm/gripper commands");
  }

private:
  void jointCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
  {
    // 🔹 Publicar solo los primeros N joints para el brazo
    std_msgs::msg::Float64MultiArray arm_cmd;
    arm_cmd.data.assign(msg->position.begin(), msg->position.begin() + 5);

    // 🔹 Publicar el último joint para el gripper (si existe)
    std_msgs::msg::Float64MultiArray gripper_cmd;
    if (msg->position.size() > 5)
      gripper_cmd.data = { msg->position.back() };

    pub_arm_->publish(arm_cmd);
    pub_gripper_->publish(gripper_cmd);
  }

  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr sub_joint_states_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr pub_arm_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr pub_gripper_;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GUIToControllerBridge>());
  rclcpp::shutdown();
  return 0;
}
