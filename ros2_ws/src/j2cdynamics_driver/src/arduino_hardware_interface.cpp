#include "j2cdynamics_driver/arduino_hardware_interface.hpp"

#include <chrono>
#include <cmath>
#include <iomanip>      
#include <sstream>
#include <stdexcept>

#include "hardware_interface/types/hardware_interface_type_values.hpp"

namespace j2cdynamics_driver
{

// ── Lifecycle: on_init ────────────────────────────────────────────────────────
// Reads parameters from the <ros2_control> block in your URDF
hardware_interface::CallbackReturn
ArduinoHardwareInterface::on_init(const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) !=
      hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  // Read params declared in the URDF <hardware> block
  port_      = info_.hardware_parameters.at("port");
  baudrate_  = std::stoi(info_.hardware_parameters.at("baudrate"));
  max_rad_s_ = std::stod(info_.hardware_parameters.at("max_rad_s"));
  ramp_step_ = std::stod(info_.hardware_parameters.at("ramp_step"));

  // Validate joint count — expect exactly 3 (left + right wheel + sweeper mode)
  if (info_.joints.size() != 3) {
    RCLCPP_FATAL(logger_, "Expected 3 joints, got %zu", info_.joints.size());
    return hardware_interface::CallbackReturn::ERROR;
  }

  left_joint_idx_ = right_joint_idx_ = sweeper_joint_idx_ = -1;
  for (size_t i = 0; i < info_.joints.size(); ++i) {
    const std::string & name = info_.joints[i].name;
    if (name.find("left")  != std::string::npos) left_joint_idx_  = static_cast<int>(i);
    if (name.find("right") != std::string::npos) right_joint_idx_ = static_cast<int>(i);
    if (name.find("sweeper") != std::string::npos) sweeper_joint_idx_ = static_cast<int>(i);
  }
  if (left_joint_idx_ < 0 || right_joint_idx_ < 0 || sweeper_joint_idx_ < 0 ||
      left_joint_idx_ == right_joint_idx_ || left_joint_idx_ == sweeper_joint_idx_ || right_joint_idx_ == sweeper_joint_idx_)
  {
    RCLCPP_FATAL(logger_,
      "Could not identify distinct 'left', 'right', and 'sweeper' joints by name "
      "(got '%s', '%s', '%s'). Rename joints or adjust the matcher.",
      info_.joints[0].name.c_str(), info_.joints[1].name.c_str(), info_.joints[2].name.c_str());
    return hardware_interface::CallbackReturn::ERROR;
  }

  RCLCPP_INFO(logger_, "on_init OK — port=%s baud=%d (left=%s right=%s sweeper=%s)",
    port_.c_str(), baudrate_,
    info_.joints[left_joint_idx_].name.c_str(),
    info_.joints[right_joint_idx_].name.c_str(),
    info_.joints[sweeper_joint_idx_].name.c_str());
  return hardware_interface::CallbackReturn::SUCCESS;
}

// ── Lifecycle: on_configure ───────────────────────────────────────────────────
// Open the serial port here (not in on_init) so it can be retried on failure
hardware_interface::CallbackReturn
ArduinoHardwareInterface::on_configure(const rclcpp_lifecycle::State &)
{
  try {
    serial_.Open(port_);
    serial_.SetBaudRate(LibSerial::BaudRate::BAUD_115200);   // match your Arduino
    serial_.SetCharacterSize(LibSerial::CharacterSize::CHAR_SIZE_8);
    serial_.SetFlowControl(LibSerial::FlowControl::FLOW_CONTROL_NONE);
    serial_.SetParity(LibSerial::Parity::PARITY_NONE);
    serial_.SetStopBits(LibSerial::StopBits::STOP_BITS_1);
  } catch (const LibSerial::OpenFailed &) {
    RCLCPP_FATAL(logger_, "Failed to open serial port: %s", port_.c_str());
    return hardware_interface::CallbackReturn::ERROR;
  }

  RCLCPP_INFO(logger_, "Waiting for Arduino to boot...");
  std::this_thread::sleep_for(std::chrono::milliseconds(2500));
  serial_.FlushIOBuffers();
  RCLCPP_INFO(logger_, "Arduino ready");

  return hardware_interface::CallbackReturn::SUCCESS;
}

// ── Lifecycle: on_activate / on_deactivate / on_cleanup ──────────────────────
hardware_interface::CallbackReturn
ArduinoHardwareInterface::on_activate(const rclcpp_lifecycle::State &)
{
  hw_cmd_left_ = hw_cmd_right_ = 0.0;
  current_left_ = current_right_ = 0.0;
  hw_vel_left_ = hw_vel_right_ = 0.0;
  hw_mode_state_ = hw_cmd_mode_ = 0.0;
  current_mode_ = last_send_mode_ = SweeperMode::Idle;
  consecutive_failures_ = 0;
  RCLCPP_INFO(logger_, "Hardware activated");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn
ArduinoHardwareInterface::on_deactivate(const rclcpp_lifecycle::State &)
{
  // Send zero velocity before deactivating
  send_command("SPEED 0.0000 0.0000\n");
  send_command("MODE IDLE\n");
  RCLCPP_INFO(logger_, "Hardware deactivated");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn
ArduinoHardwareInterface::on_cleanup(const rclcpp_lifecycle::State &)
{
  if (serial_.IsOpen()) serial_.Close();
  return hardware_interface::CallbackReturn::SUCCESS;
}

// ── Export interfaces ─────────────────────────────────────────────────────────
// Now keyed off the resolved indices so the exported names always match the
// physical wheel the hw_* variable represents.
std::vector<hardware_interface::StateInterface>
ArduinoHardwareInterface::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;

  state_interfaces.emplace_back(
    info_.joints[left_joint_idx_].name,  hardware_interface::HW_IF_VELOCITY, &hw_vel_left_);
  state_interfaces.emplace_back(
    info_.joints[left_joint_idx_].name,  hardware_interface::HW_IF_POSITION, &hw_pos_left_);
  state_interfaces.emplace_back(
    info_.joints[right_joint_idx_].name, hardware_interface::HW_IF_VELOCITY, &hw_vel_right_);
  state_interfaces.emplace_back(
    info_.joints[right_joint_idx_].name, hardware_interface::HW_IF_POSITION, &hw_pos_right_);
  state_interfaces.emplace_back(
    info_.joints[sweeper_joint_idx_].name, hardware_interface::HW_IF_POSITION, &hw_mode_state_);

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface>
ArduinoHardwareInterface::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;

  command_interfaces.emplace_back(
    info_.joints[left_joint_idx_].name,  hardware_interface::HW_IF_VELOCITY, &hw_cmd_left_);
  command_interfaces.emplace_back(
    info_.joints[right_joint_idx_].name, hardware_interface::HW_IF_VELOCITY, &hw_cmd_right_);
  command_interfaces.emplace_back(
    info_.joints[sweeper_joint_idx_].name, hardware_interface::HW_IF_POSITION, &hw_cmd_mode_);

  return command_interfaces;
}

// ── read() — poll Arduino for wheel velocities ────────────────────────────────
hardware_interface::return_type
ArduinoHardwareInterface::read(const rclcpp::Time &, const rclcpp::Duration & period)
{
  // ── ODOMETRY POLLING ─────────────────────
  double left_vel = 0.0, right_vel = 0.0;

  if (request_odometry(left_vel, right_vel)) {
    // ── success: update state + reset failure counter ──
    hw_vel_left_  = left_vel;
    hw_vel_right_ = right_vel;
    consecutive_failures_ = 0;
    
    //RCLCPP_INFO(logger_, "Left wheel=%f - Right wheel=%f", hw_vel_left_, hw_vel_right_);
  } else {
    // Returning ERROR here makes ros2_control deactivate the component. Instead
    // we keep the last good velocity (hw_vel_* already hold it) and only escalate
    // to a real fault after a sustained run of failures.
    ++consecutive_failures_;
    if (consecutive_failures_ > failure_threshold_) {
      RCLCPP_ERROR(logger_,
        "Arduino unresponsive: %d consecutive failed reads — reporting fault",
        consecutive_failures_);
      return hardware_interface::return_type::ERROR;
    }
    RCLCPP_WARN_THROTTLE(logger_, clock_, 1000,
      "Odometry read failed (%d in a row) — reusing last value",
      consecutive_failures_);
  }

  hw_pos_left_  += hw_vel_left_  * period.seconds();
  hw_pos_right_ += hw_vel_right_ * period.seconds();

  // ── MODE POLLING ─────────────────────
  SweeperMode mode;

  if (request_mode(mode)) {
    hw_mode_state_ = static_cast<double>(mode);
    if (hw_cmd_mode_ != hw_mode_state_) {
      RCLCPP_WARN(logger_,
        "Mode mismatch: commanded=%d actual=%d",
        (int)hw_cmd_mode_,
        (int)hw_mode_state_);
    }
  } else {
    // On a read failure, keep hw_mode_state_ unchanged (last good value) and
    // let the failure counter govern escalation rather than faulting on a single
    // hiccup. Note that the mode state is informational only (not used in
    // control logic), so it's less critical to have it update every cycle.
    ++consecutive_failures_;
    if (consecutive_failures_ > failure_threshold_) {
      RCLCPP_ERROR(logger_,
        "Arduino unresponsive: %d consecutive failed reads — reporting fault",
        consecutive_failures_);
      return hardware_interface::return_type::ERROR;
    }
    RCLCPP_WARN_THROTTLE(logger_, clock_, 1000,
      "Mode read failed (%d in a row) — reusing last value",
      consecutive_failures_);
  }

  return hardware_interface::return_type::OK;
}

// ── write() — send ramped velocity command to Arduino ────────────────────────
hardware_interface::return_type
ArduinoHardwareInterface::write(const rclcpp::Time &, const rclcpp::Duration &)
{
  double left  = _ramp(current_left_,  _clamp(hw_cmd_left_));
  double right = _ramp(current_right_, _clamp(hw_cmd_right_));

  current_left_  = left;
  current_right_ = right;

  std::ostringstream cmd;
  cmd << "SPEED " << std::fixed << std::setprecision(4) << left
      << " " << right << "\n";

  if (!send_command(cmd.str())) {
    // A failed write is more serious than a failed read (we can't reuse a
    // "last command"), but still let the failure counter govern escalation
    // rather than faulting on a single hiccup.
    ++consecutive_failures_;
    if (consecutive_failures_ > failure_threshold_) {
      RCLCPP_ERROR(logger_, "Repeated SPEED write failures — reporting fault");
      return hardware_interface::return_type::ERROR;
    }
    RCLCPP_WARN_THROTTLE(logger_, clock_, 1000,
      "Failed to send SPEED command (%d in a row)", consecutive_failures_);
  }

  SweeperMode desired_mode = decode_mode(hw_cmd_mode_);

  if (desired_mode != last_send_mode_) {
    std::string cmd = encode_mode_command(desired_mode);

    if (!send_command(cmd)) {
      ++consecutive_failures_;
      if (consecutive_failures_ > failure_threshold_) {
        RCLCPP_ERROR(logger_, "Repeated MODE write failures — reporting fault");
        return hardware_interface::return_type::ERROR;
      }
      RCLCPP_WARN_THROTTLE(logger_, clock_, 1000,
        "Failed to send MODE command (%d in a row)", consecutive_failures_);
    } else {
      last_send_mode_ = desired_mode;
    }
  }

  return hardware_interface::return_type::OK;
}

// ── Private helpers ───────────────────────────────────────────────────────────
bool ArduinoHardwareInterface::send_command(const std::string & cmd)
{
  try {
    serial_.Write(cmd);
    return true;
  } catch (const std::exception & e) {
    RCLCPP_ERROR(logger_, "Serial write error: %s", e.what());
    return false;
  }
}

bool ArduinoHardwareInterface::request_odometry(double & left_vel, double & right_vel)
{
  try {
    // NOTE: flushing before each request is acceptable in a strict
    // request/response protocol (clears stale partial replies). If you ever
    // move the Arduino to streaming mode, drop this and the Write below.
    serial_.FlushInputBuffer();
    serial_.Write("ODOMETRY\n");
    std::string line;

    // Short timeout: a late Arduino must not stall the control loop for long.
    // (Was 50 with a comment claiming 500 — reconcile this with your odom_rate.)
    serial_.ReadLine(line, '\n', 50);

    return parse_odometry(line, left_vel, right_vel);

  } catch (const LibSerial::ReadTimeout &) {
    // No reply this cycle — a normal transient, not an exception worth logging
    // at error level. Caller treats the false return as "reuse last value".
    return false;
  } catch (const std::exception & e) {
    RCLCPP_ERROR(logger_, "Serial read error: %s", e.what());
    return false;
  }
}

// Accept the ODOMETRY line even with leading noise/whitespace, rather than
// demanding it start exactly at index 0. Still strict about the numeric format.
bool ArduinoHardwareInterface::parse_odometry(
  const std::string & line, double & left_vel, double & right_vel)
{
  std::size_t start = line.find("ODOMETRY");
  if (start == std::string::npos) return false;

  float l = 0.0f, r = 0.0f;
  if (std::sscanf(line.c_str() + start,
        "ODOMETRY: L_vel=%f rad/s R_vel=%f rad/s", &l, &r) != 2)
  {
    return false;
  }

  // Reject obvious garbage (NaN/inf from a corrupt line)
  if (!std::isfinite(l) || !std::isfinite(r)) return false;

  left_vel  = static_cast<double>(l);
  right_vel = static_cast<double>(r);
  return true;
}

bool ArduinoHardwareInterface::request_mode(SweeperMode & mode)
{
  try {
    // NOTE: flushing before each request is acceptable in a strict
    // request/response protocol (clears stale partial replies). If you ever
    // move the Arduino to streaming mode, drop this and the Write below.
    serial_.FlushInputBuffer();
    serial_.Write("MODE\n");
    std::string line;

    // Short timeout: a late Arduino must not stall the control loop for long.
    // (Was 50 with a comment claiming 500 — reconcile this with your odom_rate.)
    serial_.ReadLine(line, '\n', 50);

    return parse_mode(line, mode);

  } catch (const LibSerial::ReadTimeout &) {
    // No reply this cycle — a normal transient, not an exception worth logging
    // at error level. Caller treats the false return as "reuse last value".
    return false;
  } catch (const std::exception & e) {
    RCLCPP_ERROR(logger_, "Serial read error: %s", e.what());
    return false;
  }
}

// Accept the MODE line even with leading noise/whitespace, rather than
// demanding it start exactly at index 0. Still strict about the numeric format.
bool ArduinoHardwareInterface::parse_mode(
  const std::string & line, SweeperMode & mode)
{
  std::size_t start = line.find("MODE:");
  if (start == std::string::npos) return false;

  std::string value = line.substr(start);

  if (value.find("IDLE") != std::string::npos) {
    mode = SweeperMode::Idle;
    return true;
  }
  if (value.find("COLLECT") != std::string::npos) {
    mode = SweeperMode::Collect;
    return true;
  }
  if (value.find("DROPOFF") != std::string::npos) {
    mode = SweeperMode::Dropoff;
    return true;
  }

  return false;
}

double ArduinoHardwareInterface::_ramp(double current, double target) const
{
  double delta = target - current;
  if (std::abs(delta) <= ramp_step_) return target;
  return current + ramp_step_ * (delta > 0.0 ? 1.0 : -1.0);
}

double ArduinoHardwareInterface::_clamp(double value) const
{
  return std::max(-max_rad_s_, std::min(max_rad_s_, value));
}

SweeperMode ArduinoHardwareInterface::decode_mode(double v) const
{
  int mode_int = static_cast<int>(std::round(v));
  switch (mode_int) {
    case 0: return SweeperMode::Idle;
    case 1: return SweeperMode::Collect;
    case 2: return SweeperMode::Dropoff;
    case 3: return SweeperMode::Fault;
    default:
      RCLCPP_WARN(logger_, "Invalid mode command value: %.2f — treating as Idle", v);
      return SweeperMode::Idle;
  }
}

std::string ArduinoHardwareInterface::encode_mode_command(SweeperMode mode) const
{
  switch (mode)
  {
    case SweeperMode::Idle:
      return "IDLE\n";

    case SweeperMode::Collect:
      return "COLLECT\n";

    case SweeperMode::Dropoff:
      return "DROPOFF\n";
  }

  return "IDLE\n";
}

}  // namespace j2cdynamics_driver

// ── Plugin registration ───────────────────────────────────────────────────────
#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  j2cdynamics_driver::ArduinoHardwareInterface,
  hardware_interface::SystemInterface)