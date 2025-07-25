#ifndef KITT_HARDWARE__KITT_HARDWARE_HPP_
#define KITT_HARDWARE__KITT_HARDWARE_HPP_

#include <memory>
#include <string>
#include <vector>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/clock.hpp"
#include "rclcpp/duration.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"

#include "kitt_hardware/visibility_control.h"

namespace kitt_hardware
{
class KittHardware : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(KittHardware);

  KITT_HARDWARE_PUBLIC
  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareInfo & info) override;

  KITT_HARDWARE_PUBLIC
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  KITT_HARDWARE_PUBLIC
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  KITT_HARDWARE_PUBLIC
  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  KITT_HARDWARE_PUBLIC
  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  KITT_HARDWARE_PUBLIC
  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  KITT_HARDWARE_PUBLIC
  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:

  double hw_start_sec_;
  double hw_stop_sec_;

  double hw_left_traction_vel_command_;
  double hw_left_traction_vel_;
  double hw_left_traction_position_;
  double hw_right_traction_vel_command_;
  double hw_right_traction_vel_;
  double hw_right_traction_position_;
  double hw_steer_axis_position_command_;
  double hw_steer_axis_vel_;
  double hw_steer_axis_position_;
};

}  // namespace kitt_hardware

#endif  // KITT_HARDWARE__KITT_HARDWARE_HPP_
