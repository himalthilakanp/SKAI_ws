#include "skai_hardware/skai_hardware.hpp"

#include "pluginlib/class_list_macros.hpp"

#include <iostream>
#include <cmath>

constexpr size_t ACTIVE_JOINTS = 3;

namespace skai_hardware
{

hardware_interface::CallbackReturn
SKAIHardware::on_init(
  const hardware_interface::HardwareComponentInterfaceParams & params
)
{
  if (
    hardware_interface::SystemInterface::on_init(params) !=
    hardware_interface::CallbackReturn::SUCCESS
  )
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  position_states_.resize(
    info_.joints.size(),
    0.0
  );

  position_commands_.resize(
    info_.joints.size(),
    0.0
  );

  driver_ =
    std::make_shared<ODriveDriver>("can0");

  if (driver_->init() != 0)
  {
    RCLCPP_ERROR(
      rclcpp::get_logger("SKAIHardware"),
      "Failed to initialize ODrive driver"
    );

    return hardware_interface::CallbackReturn::ERROR;
  }

  /*
   * CAN SEND GATE NODE
   */

  enable_node_ =
    rclcpp::Node::make_shared(
      "skai_can_gate"
    );

  enable_sub_ =
    enable_node_->create_subscription<std_msgs::msg::Bool>(
      "/can_send_enable",
      10,
      [this](std_msgs::msg::Bool::SharedPtr msg)
      {
        send_enabled_ = msg->data;

        std::cout
          << "CAN send "
          << (msg->data ? "ENABLED" : "DISABLED")
          << std::endl;
      }
    );

  enable_thread_ =
    std::thread(
      [this]()
      {
        rclcpp::spin(enable_node_);
      }
    );

  enable_thread_.detach();

  std::cout
    << "SKAI Hardware Initialized"
    << std::endl;

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn
SKAIHardware::on_activate(
  const rclcpp_lifecycle::State &
)
{
  driver_->clearErrors(
    node_ids_
  );

  usleep(100000);

  driver_->setClosedLoop(
    node_ids_
  );

  usleep(100000);

  commands_seeded_ = false;

  std::cout
    << "SKAI Hardware activated"
    << std::endl;

  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface>
SKAIHardware::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface>
    state_interfaces;

  for (
    size_t i = 0;
    i < info_.joints.size();
    i++
  )
  {
    state_interfaces.emplace_back(
      info_.joints[i].name,
      hardware_interface::HW_IF_POSITION,
      &position_states_[i]
    );
  }

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface>
SKAIHardware::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface>
    command_interfaces;

  for (
    size_t i = 0;
    i < info_.joints.size();
    i++
  )
  {
    command_interfaces.emplace_back(
      info_.joints[i].name,
      hardware_interface::HW_IF_POSITION,
      &position_commands_[i]
    );
  }

  return command_interfaces;
}

hardware_interface::return_type
SKAIHardware::read(
  const rclcpp::Time &,
  const rclcpp::Duration &
)
{
  for (
    size_t i = 0;
    i < ACTIVE_JOINTS;
    i++
  )
  {
    double position;

    if (
      driver_->readPosition(
        node_ids_[i],
        position
      )
    )
    {
      const auto & joint_name =
        info_.joints[i].name;

      if (
        joint_name == "PITCH_1" ||
        joint_name == "PITCH_2"
      )
      {
        position = -position;
      }

      position_states_[i] =
        position;
    }
  }

  position_states_[3] = 0.0;
  position_states_[4] = 0.0;
  position_states_[5] = 0.0;

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type
SKAIHardware::write(
  const rclcpp::Time &,
  const rclcpp::Duration &
)
{
  if (!send_enabled_)
  {
    return hardware_interface::return_type::OK;
  }

  if (!commands_seeded_)
  {
    for (
      size_t i = 0;
      i < position_commands_.size();
      i++
    )
    {
      position_commands_[i] =
        position_states_[i];
    }

    commands_seeded_ = true;
  }

  for (
    size_t i = 0;
    i < ACTIVE_JOINTS;
    i++
  )
  {
    double command =
      position_commands_[i];

    const auto & joint_name =
      info_.joints[i].name;

    if (
      joint_name == "PITCH_1" ||
      joint_name == "PITCH_2"
    )
    {
      command = -command;
    }

    driver_->setPosition(
      node_ids_[i],
      command
    );
  }

  return hardware_interface::return_type::OK;
}

} // namespace skai_hardware

PLUGINLIB_EXPORT_CLASS(
  skai_hardware::SKAIHardware,
  hardware_interface::SystemInterface
)