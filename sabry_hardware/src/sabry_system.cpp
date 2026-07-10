#include "sabry_hardware/sabry_system.hpp"

#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <pluginlib/class_list_macros.hpp>

#include <sstream>
#include <cmath>

namespace sabry_hardware
{

// ===================== INIT =====================
hardware_interface::CallbackReturn SabrySystem::on_init(
  const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info)
      != hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::FAILURE;
  }

  port_ = info_.hardware_parameters.at("port");

  size_t n = info_.joints.size();

  hw_positions_.resize(n, 0.0);
  hw_velocities_.resize(n, 0.0);
  hw_commands_.resize(n, 0.0);
  prev_positions_.resize(n, 0.0);
  desired_positions_.resize(n, 0.0);
  prev_velocities_cmd_.resize(n, 0.0);
  prev_errors_.resize(n, 0.0);
  raw_motor_positions_.resize(n, 0.0);
  prev_raw_deg_.resize(n, 0.0);
  integral_errors_.resize(n, 0.0);

  encoder_offsets_.resize(n, 0.0);
  continuous_positions_.resize(n, 0.0);
  first_read_.resize(n, true);

  return hardware_interface::CallbackReturn::SUCCESS;
}

// ===================== ACTIVATE =====================
hardware_interface::CallbackReturn SabrySystem::on_activate(
  const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(rclcpp::get_logger("SabrySystem"), "Activating hardware...");

  try {
    serial_.Open(port_);
    serial_.SetBaudRate(LibSerial::BaudRate::BAUD_115200);
    serial_.FlushIOBuffers();
  } catch (const std::exception &e) {
    RCLCPP_FATAL(rclcpp::get_logger("SabrySystem"),
                 "Failed to open serial: %s", e.what());
    return hardware_interface::CallbackReturn::FAILURE;
  }
  node_ = rclcpp::Node::make_shared("sabry_hw_tool_subscriber");

  tool_sub_ = node_->create_subscription<std_msgs::msg::Int32>(
    "/tool_changer/command",
    10,
    [this](const std_msgs::msg::Int32::SharedPtr msg)
    {
      latest_tool_cmd_ = msg->data;
      new_tool_cmd_ = true;
    });

  executor_.add_node(node_);
  executor_thread_ = std::thread([this]() {
      executor_.spin();
  });

  encoder_offsets_ = {270.79, 201.70, 313.86, 180.0, 190.46};  

  // Wait for a valid encoder packet before starting
  // This prevents the 0 -> real_angle jump on first read()
  int attempts = 0;
  while (attempts < 50) {
    read_serial_encoders();
    if (hw_positions_[0] != 0.0) break; // got a real reading
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
    attempts++;
  }

  // NOW initialize prev_positions to the real encoder state
  for (size_t i = 0; i < hw_positions_.size(); i++) {
    hw_velocities_[i]  = 0.0;
    hw_commands_[i]    = 0.0;
    prev_positions_[i] = hw_positions_[i]; // ← real position, not 0.0
    desired_positions_[i] = hw_positions_[i];  // start from real pose
    prev_velocities_cmd_[i] = 0.0;
    prev_errors_[i] = 0.0;
  }

  prev_read_time_ = clock_.now();
  first_read_done_ = false; // add this flag to your header

  return hardware_interface::CallbackReturn::SUCCESS;
}

// ===================== DEACTIVATE =====================
hardware_interface::CallbackReturn SabrySystem::on_deactivate(
  const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(rclcpp::get_logger("SabrySystem"), "Deactivating hardware...");

  if (serial_.IsOpen())
    serial_.Close();

  return hardware_interface::CallbackReturn::SUCCESS;
}

// ===================== SERIAL READ (SAFE) =====================
void SabrySystem::read_serial_encoders()
{
  std::string line;

  if (!serial_.IsDataAvailable())
    return;

  try
  {
    serial_.ReadLine(line, '\n', 5);
  }
  catch (const LibSerial::ReadTimeout &)
  {
    return;
  }
  catch (const std::exception &e)
  {
    RCLCPP_WARN_THROTTLE(
      rclcpp::get_logger("SabrySystem"), clock_, 2000,
      "Serial error: %s", e.what());
    return;
  }

  if (line.empty())
    return;

  std::stringstream ss(line);
  std::string token;

  std::vector<double> temp(hw_positions_.size(), 0.0);
  size_t i = 0;

  while (std::getline(ss, token, ','))
  {
    
    // ===== GRIPPER FEEDBACK =====
    if (token.find("GRIP:") != std::string::npos)
    {
      try {
        gripper_feedback_deg_ = std::stod(token.substr(5));
      } catch (...) {}
      continue;
    }

    double raw_deg = 0.0;
    double corrected_deg = 0.0;
    
    try
    {
      raw_deg = std::stod(token);            // 0–360 from encoder
      corrected_deg = raw_deg - encoder_offsets_[i];
    }
    catch (...)
    {
      return; // ignore corrupted packet
    }

    // ================= NORMAL JOINTS =================
    if (i < 3)
    {
      if (i == 1)
        raw_motor_positions_[i] = -corrected_deg * M_PI / 180.0;
      else
        raw_motor_positions_[i] = corrected_deg * M_PI / 180.0;

      i++;
      continue;
    }

    // ================= CONTINUOUS JOINTS =================
    if (first_read_[i])
    {
      continuous_positions_[i] = corrected_deg * M_PI / 180.0;
      prev_raw_deg_[i] = corrected_deg;

      raw_motor_positions_[i] = continuous_positions_[i];

      first_read_[i] = false;
      i++;
      continue;
    }

    double delta = corrected_deg - prev_raw_deg_[i];

    // unwrap to [-180, 180]
    while (delta > 180.0)  delta -= 360.0;
    while (delta < -180.0) delta += 360.0;

    // REJECT unrealistic jumps 
    double max_delta = 70.0;  // deg per cycle (adjust based on your speed)

    if (std::fabs(delta) > max_delta)
    {
      // skip this update → likely corrupted or skipped frame
      return;
    }

    // accumulate
    continuous_positions_[i] += delta * M_PI / 180.0;
    prev_raw_deg_[i] = corrected_deg;

    raw_motor_positions_[i] = continuous_positions_[i];

    i++;
  }

  if (i == 0)
    return; // incomplete packet

  // ================= DIFFERENTIAL KINEMATICS =================

  // ONLY after full parsing is done:
  size_t m1 = 4;
  size_t m2 = 3;

  double ratio = 39.0 / 29.0;

  double theta_m1 = raw_motor_positions_[m1];
  double theta_m2 = raw_motor_positions_[m2];

  double theta_yaw   = -3.5 * ((theta_m1 - theta_m2) / 8.0);
  double theta_pitch = -5.2 * (theta_m1 + theta_m2) / (8.0 * ratio);

  // double theta_yaw   = ((theta_m1 + theta_m2) / 8.0);
  // double theta_pitch = (theta_m1 - theta_m2) / (8.0 * ratio);

  // overwrite ONLY final outputs
  hw_positions_ = raw_motor_positions_;
  hw_positions_[4] = theta_yaw;
  hw_positions_[3] = theta_pitch;
  
  // RCLCPP_INFO_THROTTLE(
  //   rclcpp::get_logger("SabrySystem"),
  //   clock_,
  //   2000,
  //   "J3=%.3f rad | J4=%.3f rad",
  //   hw_positions_[3],
  //   hw_positions_[4]);
}

// ===================== READ =====================
hardware_interface::return_type SabrySystem::read(
  const rclcpp::Time &,
  const rclcpp::Duration &)
{
  // executor_.spin_some(std::chrono::nanoseconds(0));
  read_serial_encoders();
  RCLCPP_DEBUG_THROTTLE(
    rclcpp::get_logger("SabrySystem"), clock_, 2000,
    "read() running");

  rclcpp::Time current_time = clock_.now();
  double dt = (current_time - prev_read_time_).seconds();

  // Skip velocity computation on first cycle entirely
  if (!first_read_done_) {
    for (size_t i = 0; i < hw_positions_.size(); i++)
      prev_positions_[i] = hw_positions_[i];
    prev_read_time_ = current_time;
    first_read_done_ = true;
    return hardware_interface::return_type::OK;
  }

  if (dt < 1e-4) dt = 1e-4; // floor at 0.1ms, not 1us

  for (size_t i = 0; i < hw_positions_.size(); i++) {
    hw_velocities_[i]  = (hw_positions_[i] - prev_positions_[i]) / dt;
    prev_positions_[i] = hw_positions_[i];
    // if (std::fabs(hw_velocities_[i]) < 0.2)
    //   hw_velocities_[i] = 0.0;
  }

  gripper_position_ = gripper_feedback_deg_ * M_PI / 180.0;
  RCLCPP_INFO_THROTTLE(
    rclcpp::get_logger("SabrySystem"),
    clock_,
    500,
    "GRIP FB DEG=%.2f | RAD=%.2f",
    gripper_feedback_deg_,
    gripper_position_);
  static auto last = std::chrono::steady_clock::now();
  auto now = std::chrono::steady_clock::now();
  double dt_loop = std::chrono::duration<double>(now - last).count();
  last = now;

  RCLCPP_INFO_THROTTLE(
    rclcpp::get_logger("SabrySystem"),
    clock_,
    1000,
    "Loop Hz: %.1f", 1.0 / dt_loop);

  prev_read_time_ = current_time;
  return hardware_interface::return_type::OK;
}

// ===================== WRITE =====================
hardware_interface::return_type SabrySystem::write(
  const rclcpp::Time &,
  const rclcpp::Duration &)
{
  static rclcpp::Time prev_time = clock_.now();
  rclcpp::Time current_time = clock_.now();

  double dt = (current_time - prev_time).seconds();
  if (dt < 1e-4) dt = 1e-4;

  prev_time = current_time;

  std::vector<double> controlled_vel(hw_commands_.size(), 0.0);

  std::vector<double> Kp = {
    6.0, // joint 0
    10.0, // joint 1
    3.0, // joint 2
    1.0, // joint 3 (pitch differential)
    1.0  // joint 4 (yaw differential)
  };

  std::vector<double> Kd = {
    5.0, // joint 0
    5.0, // joint 1
    4.0, // joint 2
    2.0, // joint 3
    2.0  // joint 4
  };

  std::vector<double> Ki = {
    0.2, // joint 0
    0.1, // joint 1
    0.1, // joint 2
    0.1, // joint 3
    0.1  // joint 4
  };

  double max_acc = 5.0; // rad/s²

  for (size_t i = 0; i < hw_commands_.size(); i++)
  {
    double desired_vel = hw_commands_[i];

    // ===== 1. Integrate desired velocity =====
    desired_positions_[i] += desired_vel * dt;

    // ===== 2. Position error =====
    double error = desired_positions_[i] - hw_positions_[i];

    // ===== INTEGRAL =====
    integral_errors_[i] += error * dt;

    double integral_limit = 1.0;

    // anti-windup clamp
    integral_errors_[i] = std::max(-integral_limit,
                            std::min(integral_errors_[i], integral_limit));

    // ===== DERIVATIVE =====
    double error_dot = (error - prev_errors_[i]) / dt;

    // ===== PID =====
    double v = desired_vel
          + Kp[i] * error
          + Ki[i] * integral_errors_[i]
          + Kd[i] * error_dot;

    // ===== 5. Acceleration limiting =====
    double dv = v - prev_velocities_cmd_[i];
    double max_dv = max_acc * dt;

    if (dv > max_dv)
        dv = max_dv;

    if (dv < -max_dv)
        dv = -max_dv;

    v = prev_velocities_cmd_[i] + dv;

    // ===== 6. Store previous values =====
    prev_velocities_cmd_[i] = v;
    prev_errors_[i] = error;

    controlled_vel[i] = v;
  }
  std::stringstream msg;

  size_t yaw = 4;
  size_t pitch = 3;

  double ratio = 39.0 / 29.0;

  // ===== MAX VELOCITY =====
  std::vector<double> max_vel_deg = {
    150.0,  // joint 0
    250.0,  // joint 1
    200.0,  // joint 2
    100.0,  // pitch (joint 3)
    100.0   // yaw (joint 4)
  };

  std::vector<double> max_vel_rad(max_vel_deg.size());
  for (size_t i = 0; i < max_vel_deg.size(); i++)
  {
    max_vel_rad[i] = max_vel_deg[i] * M_PI / 180.0;
  }

  std::vector<double> limited_vel(hw_commands_.size(), 0.0);

  // ===== LIMIT JOINT VELOCITIES =====
  for (size_t i = 0; i < hw_commands_.size(); i++)
  {
    double v = controlled_vel[i];

    double max_v = max_vel_rad[i];

    if (v > max_v)
      v = max_v;
    else if (v < -max_v)
      v = -max_v;

    limited_vel[i] = v;

    msg << (v * 180.0 / M_PI) << ",";
  }

  // ===== DIFFERENTIAL (velocity form) =====
  double yaw_vel   = limited_vel[yaw];
  double pitch_vel = limited_vel[pitch];

  double m1_vel = 1.0 * (pitch_vel - ratio * yaw_vel);
  double m2_vel = 1.0 * (pitch_vel + ratio * yaw_vel);

  double m1_deg = m1_vel * 180.0 / M_PI;
  double m2_deg = m2_vel * 180.0 / M_PI;

  msg << "M1:" << m1_deg << ",";
  msg << "M2:" << m2_deg;

  // msg << "M1:" << 0.0 << ",";
  // msg << "M2:" << 0.0;
  if (new_tool_cmd_)
  {
    msg << ",TOOL:" << latest_tool_cmd_;;
    new_tool_cmd_ = false;
  }

  double gripper_deg = gripper_command_ * 180.0 / M_PI;
  msg << ",GRIP:" << gripper_deg;

  msg << "\n";

  try
  {
    serial_.Write(msg.str());
  }
  catch (const std::exception &e)
  {
    RCLCPP_ERROR(rclcpp::get_logger("SabrySystem"),
                 "Serial write failed: %s", e.what());
    return hardware_interface::return_type::ERROR;
  }

  // ===== LOG ACTUAL VELOCITY =====
  RCLCPP_INFO_THROTTLE(
    rclcpp::get_logger("SabrySystem"),
    clock_,
    500,
    "VEL(deg/s): [%.1f %.1f %.1f %.1f %.1f] | M1=%.1f M2=%.1f | TOOL=%d | GRIP=%.2f",
    limited_vel[0] * 180.0 / M_PI,
    limited_vel[1] * 180.0 / M_PI,
    limited_vel[2] * 180.0 / M_PI,
    limited_vel[3] * 180.0 / M_PI,
    limited_vel[4] * 180.0 / M_PI,
    m1_deg,
    m2_deg, 
    latest_tool_cmd_,
    gripper_deg
  );

  return hardware_interface::return_type::OK;
}

// ===================== INTERFACES =====================
std::vector<hardware_interface::StateInterface>
SabrySystem::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> out;

  for (size_t i = 0; i < hw_positions_.size(); i++)
  {
    if (i == 5)
    {
      // Use gripper feedback ONLY
      out.emplace_back(info_.joints[i].name,
                      hardware_interface::HW_IF_POSITION,
                      &gripper_position_);
    }
    else
    {
      out.emplace_back(info_.joints[i].name,
                      hardware_interface::HW_IF_POSITION,
                      &hw_positions_[i]);
                      
      out.emplace_back(info_.joints[i].name,
                    hardware_interface::HW_IF_VELOCITY,
                    &hw_velocities_[i]);
    }

  }

  return out;
}

std::vector<hardware_interface::CommandInterface>
SabrySystem::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> out;

  for (size_t i = 0; i < hw_commands_.size(); i++)
  {
    if (i == 5)
    {
      out.emplace_back(info_.joints[i].name,
                      hardware_interface::HW_IF_POSITION,
                      &gripper_command_);
    }
    else
    {
      out.emplace_back(info_.joints[i].name,
                      hardware_interface::HW_IF_VELOCITY,
                      &hw_commands_[i]);
    }
  }


  return out;
}

} // namespace sabry_hardware

PLUGINLIB_EXPORT_CLASS(
  sabry_hardware::SabrySystem,
  hardware_interface::SystemInterface)
