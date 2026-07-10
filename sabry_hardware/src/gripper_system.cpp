#include "rclcpp/rclcpp.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "pluginlib/class_list_macros.hpp"

#include "std_msgs/msg/int32.hpp"

using CallbackReturn =
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

namespace gripper_hardware
{

class GripperSystem : public hardware_interface::SystemInterface
{
public:

  CallbackReturn on_init(
    const hardware_interface::HardwareInfo & info) override
  {
    if (hardware_interface::SystemInterface::on_init(info) != CallbackReturn::SUCCESS)
    {
      return CallbackReturn::ERROR;
    }

    node_ = rclcpp::Node::make_shared("gripper_hardware_interface");
    

    gripper_cmd_pub_ =
      node_->create_publisher<std_msgs::msg::Int32>("/gripper_goal", 10);

    gripper_state_sub_ =
      node_->create_subscription<std_msgs::msg::Int32>(
        "/gripper_pos",
        10,
        std::bind(&GripperSystem::gripper_callback, this, std::placeholders::_1));
    
    executor_.add_node(node_);

    ros_thread_ = std::thread([this]() {
      executor_.spin();
    });
    RCLCPP_INFO(node_->get_logger(), "GripperSystem initialized");

    return CallbackReturn::SUCCESS;
  }

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override
  {
    std::vector<hardware_interface::StateInterface> state_interfaces;

    state_interfaces.emplace_back(
      hardware_interface::StateInterface(
        "left_tool_joint",
        hardware_interface::HW_IF_POSITION,
        &gripper_position_));

    return state_interfaces;
  }

  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override
  {
    std::vector<hardware_interface::CommandInterface> command_interfaces;

    command_interfaces.emplace_back(
      hardware_interface::CommandInterface(
        "left_tool_joint",
        hardware_interface::HW_IF_POSITION,
        &gripper_command_));

    return command_interfaces;
  }

  CallbackReturn on_activate(
    const rclcpp_lifecycle::State &) override
  {
    gripper_command_ = gripper_position_;

    RCLCPP_INFO(node_->get_logger(), "GripperSystem activated");

    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State &) override
  {
    RCLCPP_INFO(node_->get_logger(), "GripperSystem deactivated");

    return CallbackReturn::SUCCESS;
  }

  hardware_interface::return_type read(
    const rclcpp::Time &,
    const rclcpp::Duration &) override
  {
    // rclcpp::spin_some(node_);

    // convert degrees -> radians
    gripper_position_ = last_feedback_deg_ * M_PI / 180.0;

    return hardware_interface::return_type::OK;
  }

  hardware_interface::return_type write(
    const rclcpp::Time &,
    const rclcpp::Duration &) override
  {
    std_msgs::msg::Int32 msg;

    // convert radians -> degrees
    msg.data = gripper_command_ * 180.0 / M_PI;

    gripper_cmd_pub_->publish(msg);

    return hardware_interface::return_type::OK;
  }

private:

  double gripper_position_ = 0.0;
  double gripper_command_ = 0.0;

  int last_feedback_deg_ = 0;

  rclcpp::Node::SharedPtr node_;

  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr gripper_cmd_pub_;
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr gripper_state_sub_;
  rclcpp::executors::SingleThreadedExecutor executor_;
  std::thread ros_thread_;

  void gripper_callback(const std_msgs::msg::Int32::SharedPtr msg)
  {
    last_feedback_deg_ = msg->data;
  }
};

}  // namespace gripper_hardware


PLUGINLIB_EXPORT_CLASS(
  gripper_hardware::GripperSystem,
  hardware_interface::SystemInterface)
