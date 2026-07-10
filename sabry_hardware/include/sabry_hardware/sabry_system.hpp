#ifndef SABRY_HARDWARE__SABRY_SYSTEM_HPP_
#define SABRY_HARDWARE__SABRY_SYSTEM_HPP_

#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp_lifecycle/state.hpp"

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int32.hpp>
#include <vector>
#include <string>
#include <thread>
#include <atomic>
#include <mutex>

// Serial
#include <libserial/SerialPort.h>

namespace sabry_hardware
{

class SabrySystem : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(SabrySystem)

  // ===================== Lifecycle =====================
  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareInfo & info) override;

  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  // ===================== Interfaces =====================
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  // ===================== Read / Write =====================
  hardware_interface::return_type read(
    const rclcpp::Time & time,
    const rclcpp::Duration & period) override;

  hardware_interface::return_type write(
    const rclcpp::Time & time,
    const rclcpp::Duration & period) override;

private:

  // ===================== Serial =====================
  LibSerial::SerialPort serial_;
  std::string port_;

  // ===================== Joint Data =====================
  std::vector<double> hw_positions_;
  std::vector<double> hw_velocities_;
  std::vector<double> hw_commands_;
  std::vector<double> prev_positions_;
  std::vector<double> desired_positions_;
  std::vector<double> prev_velocities_cmd_;
  std::vector<double> prev_errors_;
  std::vector<double> integral_errors_;

  std::vector<double> raw_motor_positions_;
  std::vector<double> prev_raw_deg_;

  std::thread serial_thread_;
  std::atomic<bool> running_{false};

  std::mutex data_mutex_;

  std::vector<double> latest_positions_;
  bool new_data_{false};
  bool first_read_done_{false};

  // ===================== Encoder Handling =====================
  std::vector<double> encoder_offsets_;
  std::vector<double> continuous_positions_;
  std::vector<bool> first_read_;
  std::thread executor_thread_;
  int32_t latest_tool_cmd_ = 0;
  bool new_tool_cmd_ = false;
  double gripper_position_ = 0.0;
  double gripper_command_  = 0.0;
  double gripper_feedback_deg_ = 0.0;

  // ===================== Timing =====================
  rclcpp::Time prev_read_time_;
  rclcpp::Clock clock_{RCL_STEADY_TIME};
  rclcpp::Node::SharedPtr node_;
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr tool_sub_;
  rclcpp::executors::SingleThreadedExecutor executor_;

  // ===================== Helpers =====================
  void read_serial_encoders();
};

}  // namespace sabry_hardware

#endif  // SABRY_HARDWARE__SABRY_SYSTEM_HPP_