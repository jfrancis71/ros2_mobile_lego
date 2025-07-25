#ifndef ALFIE_HARDWARE__ALFIE_HARDWARE_HPP_
#define ALFIE_HARDWARE__ALFIE_HARDWARE_HPP_

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

#include "alfie_hardware/visibility_control.h"

namespace alfie_hardware
{
class AlfieHardware : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(AlfieHardware);

  ALFIE_HARDWARE_PUBLIC
  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareInfo & info) override;

  ALFIE_HARDWARE_PUBLIC
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  ALFIE_HARDWARE_PUBLIC
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  ALFIE_HARDWARE_PUBLIC
  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  ALFIE_HARDWARE_PUBLIC
  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  ALFIE_HARDWARE_PUBLIC
  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  ALFIE_HARDWARE_PUBLIC
  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:

  double hw_start_sec_;
  double hw_stop_sec_;

  double hw_traction_vel_command_;
  double hw_traction_vel_;
  double hw_traction_position_;
  double hw_steer_axis_position_command_;
  double hw_steer_axis_vel_;
  double hw_steer_axis_position_;
};

}  // namespace alfie_hardware

#endif  // ALFIE_HARDWARE__ALFIE_HARDWARE_HPP_
