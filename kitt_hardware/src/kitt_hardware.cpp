#include "kitt_hardware/kitt_hardware.hpp"

#include <chrono>
#include <cmath>
#include <cstddef>
#include <limits>
#include <memory>
#include <vector>
#include <map>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

#include "BrickPi3.cpp"

const double MATH_PI = 3.141592653589793;

BrickPi3 brickpi3;

const double traction_gear_ratio = -1.0;
const double pivot_seperation = 2.0;
const double rack_tooth_spacing = 3.0/8.0;
const double num_pinion_teeth = 12.0;


namespace kitt_hardware
{
hardware_interface::CallbackReturn KittHardware::on_init(
  const hardware_interface::HardwareInfo & info)
{
  if (
    hardware_interface::SystemInterface::on_init(info) !=
    hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  hw_start_sec_ = std::stod(info_.hardware_parameters["hw_start_duration_sec"]);
  hw_stop_sec_ = std::stod(info_.hardware_parameters["hw_stop_duration_sec"]);

  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> KittHardware::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  state_interfaces.emplace_back(hardware_interface::StateInterface(
      "left_traction_wheel", hardware_interface::HW_IF_VELOCITY, &hw_left_traction_vel_));
  state_interfaces.emplace_back(hardware_interface::StateInterface(
      "left_traction_wheel", hardware_interface::HW_IF_POSITION, &hw_left_traction_position_));
  state_interfaces.emplace_back(hardware_interface::StateInterface(
      "right_traction_wheel", hardware_interface::HW_IF_VELOCITY, &hw_right_traction_vel_));
  state_interfaces.emplace_back(hardware_interface::StateInterface(
      "right_traction_wheel", hardware_interface::HW_IF_POSITION, &hw_right_traction_position_));
  state_interfaces.emplace_back(hardware_interface::StateInterface(
      "steer_axis", hardware_interface::HW_IF_VELOCITY, &hw_steer_axis_vel_));
  state_interfaces.emplace_back(hardware_interface::StateInterface(
      "steer_axis", hardware_interface::HW_IF_POSITION, &hw_steer_axis_position_));

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> KittHardware::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  command_interfaces.emplace_back(hardware_interface::CommandInterface(
      "left_traction_wheel", hardware_interface::HW_IF_VELOCITY, &hw_left_traction_vel_command_));
  command_interfaces.emplace_back(hardware_interface::CommandInterface(
      "right_traction_wheel", hardware_interface::HW_IF_VELOCITY, &hw_right_traction_vel_command_));
  command_interfaces.emplace_back(hardware_interface::CommandInterface(
      "steer_axis", hardware_interface::HW_IF_POSITION, &hw_steer_axis_position_command_));

  return command_interfaces;
}

hardware_interface::CallbackReturn KittHardware::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("KittHardware"), "Activating ...please wait...");

  RCLCPP_INFO(
    rclcpp::get_logger("BrickPi3MotorsHardware"),
    "Resetting position joint on interface PORT_B.");
  int stalled = 0;
  brickpi3.set_motor_power(PORT_B, 40);
  rclcpp::sleep_for(std::chrono::milliseconds(100));
  while (stalled==0) {
    rclcpp::sleep_for(std::chrono::milliseconds(10));
    uint8_t state;
    int8_t power = 0;
    int32_t position = 0;
    int16_t dps = 0;
    brickpi3.get_motor_status(PORT_B, state, power, position, dps);
    if (dps < 30.0) { //30
      stalled = 1;
      brickpi3.set_motor_power(PORT_B, 0.0);
    }
  }
  rclcpp::sleep_for(std::chrono::milliseconds(10));
  brickpi3.set_motor_position_relative(PORT_B, -125.0);
  rclcpp::sleep_for(std::chrono::milliseconds(3000));
  brickpi3.set_motor_power(PORT_B, 0);
  brickpi3.offset_motor_encoder(PORT_B, brickpi3.get_motor_encoder(PORT_B));

  for (auto i = 0; i < hw_start_sec_; i++)
  {
    rclcpp::sleep_for(std::chrono::seconds(1));
    RCLCPP_INFO(
      rclcpp::get_logger("KittHardware"), "%.1f seconds left...", hw_start_sec_ - i);
  }

  RCLCPP_INFO(rclcpp::get_logger("KittHardware"), "Successfully activated!");

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn KittHardware::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("KittHardware"), "Deactivating ...please wait...");
  brickpi3.set_motor_dps(PORT_A, 0.0);
  brickpi3.set_motor_dps(PORT_D, 0.0);
  brickpi3.set_motor_dps(PORT_B, 0.0);

  for (auto i = 0; i < hw_stop_sec_; i++)
  {
    rclcpp::sleep_for(std::chrono::seconds(1));
    RCLCPP_INFO(
      rclcpp::get_logger("KittHardware"), "%.1f seconds left...", hw_stop_sec_ - i);
  }

  RCLCPP_INFO(rclcpp::get_logger("KittHardware"), "Successfully deactivated!");

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type KittHardware::read(
  const rclcpp::Time &, const rclcpp::Duration & period)
{
  uint8_t state_A;
  uint8_t state_D;
  uint8_t state_B;
  int8_t power_A;
  int8_t power_D;
  int8_t power_B;
  int32_t motor_position_degrees_A;
  int32_t motor_position_degrees_D;
  int32_t motor_position_degrees_B;
  int16_t motor_dps_A;
  int16_t motor_dps_D;
  int16_t motor_dps_B;
  int32_t position_A;
  int32_t position_D;
  int32_t position_B;
  int16_t dps;
  double radps_A;
  double radps_D;
  double position;

  brickpi3.get_motor_status(PORT_A, state_A, power_A, motor_position_degrees_A, motor_dps_A);
  brickpi3.get_motor_status(PORT_D, state_D, power_D, motor_position_degrees_D, motor_dps_D);
  brickpi3.get_motor_status(PORT_B, state_B, power_B, motor_position_degrees_B, motor_dps_B);
  position_A = motor_position_degrees_A*(2.0*MATH_PI/360.0);
  position_D = motor_position_degrees_D*(2.0*MATH_PI/360.0);
  radps_A = motor_dps_A*(2.0*MATH_PI/360.0);
  radps_D = motor_dps_D*(2.0*MATH_PI/360.0);
  hw_left_traction_vel_ = radps_A/traction_gear_ratio;
  hw_right_traction_vel_ = radps_D/traction_gear_ratio;
  hw_left_traction_position_ = position_A/traction_gear_ratio;
  hw_right_traction_position_ = position_D/traction_gear_ratio;

  double actuator_position = -motor_position_degrees_B*(2.0*MATH_PI/360.0);
  hw_steer_axis_position_ = asin(actuator_position * (rack_tooth_spacing * num_pinion_teeth) / (2*MATH_PI*pivot_seperation));
  double actuator_position_vel_rads = -motor_dps_B*(2.0*MATH_PI/360.0);
  hw_steer_axis_vel_ = actuator_position_vel_rads * (rack_tooth_spacing * num_pinion_teeth) / (2*MATH_PI*pivot_seperation);

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type KittHardware::write(
  const rclcpp::Time &, const rclcpp::Duration & /*period*/)
{
  RCLCPP_DEBUG(
    rclcpp::get_logger("KittHardware"),
    "Command interface traction has value %f,%f.",
    hw_left_traction_vel_command_, hw_right_traction_vel_command_);
  double left_dps = hw_left_traction_vel_command_*360.0/(2.0*MATH_PI);
  double right_dps = hw_right_traction_vel_command_*360.0/(2.0*MATH_PI);
  double left_motor_dps = left_dps * traction_gear_ratio;
  double right_motor_dps = right_dps * traction_gear_ratio;
  brickpi3.set_motor_dps(PORT_A, left_motor_dps);
  brickpi3.set_motor_dps(PORT_D, right_motor_dps);

  double actuator_rotation_rads = 2 * MATH_PI * pivot_seperation * sin(hw_steer_axis_position_command_) /
    (rack_tooth_spacing * num_pinion_teeth);
  double actuator_steering_degrees = -actuator_rotation_rads*360.0/(2.0*MATH_PI);
  double limit_motor_degrees = std::max(std::min(actuator_steering_degrees, 100.0), -100.0);
  
  RCLCPP_DEBUG(
    rclcpp::get_logger("KittHardware"),
    "received command %f, setting actuator %f.", hw_steer_axis_position_command_,
    limit_motor_degrees);
  brickpi3.set_motor_position(PORT_B, -limit_motor_degrees);

  return hardware_interface::return_type::OK;
}

}  // namespace kitt_hardware

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  kitt_hardware::KittHardware, hardware_interface::SystemInterface)
