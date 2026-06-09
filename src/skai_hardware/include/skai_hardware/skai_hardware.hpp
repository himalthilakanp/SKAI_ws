#ifndef SKAI_HARDWARE_HPP
#define SKAI_HARDWARE_HPP

#include <vector>
#include <atomic>
#include <thread>
#include <memory>

#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/types/hardware_component_interface_params.hpp"

#include "rclcpp/macros.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"

#include "skai_hardware/odrive_driver.hpp"

namespace skai_hardware
{

class SKAIHardware
  : public hardware_interface::SystemInterface
{
public:

  RCLCPP_SHARED_PTR_DEFINITIONS(
    SKAIHardware
  )

  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareComponentInterfaceParams & params
  ) override;

  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state
  ) override;

  std::vector<hardware_interface::StateInterface>
  export_state_interfaces() override;

  std::vector<hardware_interface::CommandInterface>
  export_command_interfaces() override;

  hardware_interface::return_type read(
    const rclcpp::Time & time,
    const rclcpp::Duration & period
  ) override;

  hardware_interface::return_type write(
    const rclcpp::Time & time,
    const rclcpp::Duration & period
  ) override;

private:

  /*
   * DRIVER
   */

  std::shared_ptr<ODriveDriver> driver_;

  /*
   * JOINT STATES / COMMANDS
   */

  std::vector<double> position_states_;

  std::vector<double> position_commands_;

  /*
   * ODRIVE NODE IDS
   */

  std::vector<int> node_ids_ =
  {
    21,
    22,
    23,
    24,
    25,
    26
  };

  /*
   * STARTUP FLAGS
   */

  bool commands_seeded_ = false;

  /*
   * CAN SEND GATE
   */

  std::atomic<bool> send_enabled_{false};

  rclcpp::Node::SharedPtr enable_node_;

  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr enable_sub_;

  std::thread enable_thread_;
};

} // namespace skai_hardware

#endif