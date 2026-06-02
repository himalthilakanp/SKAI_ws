#include "skai_hardware/skai_hardware.hpp"

#include "pluginlib/class_list_macros.hpp"

#include <iostream>
#include <cmath>
#include <cstring>
#include <errno.h>

namespace skai_hardware
{

hardware_interface::CallbackReturn
SKAIHardware::on_init(
  const hardware_interface::HardwareComponentInterfaceParams & params
)
{

  if (
    hardware_interface::SystemInterface::on_init(params)
    != hardware_interface::CallbackReturn::SUCCESS
  )
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  /*
   * INITIALIZE JOINT STORAGE
   */

  position_states_.resize(
    info_.joints.size(),
    0.0
  );

  position_commands_.resize(
    info_.joints.size(),
    0.0
  );

  std::cout << std::endl;
  std::cout << "SKAI Hardware Initialized"
            << std::endl;

  /*
   * SOCKETCAN INITIALIZATION
   */

  can_socket_ = socket(
    PF_CAN,
    SOCK_RAW,
    CAN_RAW
  );

  if (can_socket_ < 0)
  {

    std::cout
      << "Failed to create CAN socket"
      << std::endl;

    return hardware_interface::CallbackReturn::ERROR;
  }

  strcpy(
    ifr_.ifr_name,
    "can0"
  );

  ioctl(
    can_socket_,
    SIOCGIFINDEX,
    &ifr_
  );

  addr_.can_family = AF_CAN;

  addr_.can_ifindex = ifr_.ifr_ifindex;

  if (
    bind(
      can_socket_,
      (struct sockaddr *)&addr_,
      sizeof(addr_)
    ) < 0
  )
  {

    std::cout
      << "Failed to bind CAN socket"
      << std::endl;

    return hardware_interface::CallbackReturn::ERROR;
  }

  std::cout
    << "SocketCAN initialized"
    << std::endl;

  /*
   * CAN SEND GATE NODE
   * Subscribes to /can_send_enable (std_msgs/Bool).
   * write() calls spin_some() each cycle to process callbacks.
   */

  enable_node_ = rclcpp::Node::make_shared("skai_can_gate");

  enable_sub_ = enable_node_->create_subscription<std_msgs::msg::Bool>(
    "/can_send_enable",
    10,
    [this](std_msgs::msg::Bool::SharedPtr msg) {
      send_enabled_ = msg->data;
      std::cout
        << "CAN send "
        << (msg->data ? "ENABLED" : "DISABLED")
        << std::endl;
    }
  );

  enable_thread_ = std::thread(
    [this]() { rclcpp::spin(enable_node_); }
  );

  enable_thread_.detach();

  return hardware_interface::CallbackReturn::SUCCESS;
}



/*
 * CLEAR ODRIVE ERRORS
 */

void SKAIHardware::clear_errors()
{

  for (int node_id : node_ids_)
  {

    struct can_frame frame;

    frame.can_id =
      (node_id << 5)
      | 0x018;

    frame.can_dlc = 0;

    ::write(
      can_socket_,
      &frame,
      sizeof(frame)
    );
  }

  std::cout
    << "ODrive errors cleared"
    << std::endl;
}



/*
 * SET CLOSED LOOP CONTROL
 */

void SKAIHardware::set_closed_loop()
{

  for (int node_id : node_ids_)
  {

    struct can_frame frame;

    frame.can_id =
      (node_id << 5)
      | 0x007;

    frame.can_dlc = 4;

    uint32_t state = 8;

    memcpy(
      &frame.data[0],
      &state,
      sizeof(uint32_t)
    );

    ::write(
      can_socket_,
      &frame,
      sizeof(frame)
    );
  }

  std::cout
    << "ODrive CLOSED_LOOP sent"
    << std::endl;
}



/*
 * EXPORT STATE INTERFACES
 */

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



/*
 * EXPORT COMMAND INTERFACES
 */

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



/*
 * READ
 */

hardware_interface::return_type
SKAIHardware::read(
  const rclcpp::Time & time,
  const rclcpp::Duration & period
)
{

  /*
   * REQUEST ENCODER ESTIMATES FROM ALL NODES
   */

  for (size_t i = 0; i < node_ids_.size(); i++)
  {

    struct can_frame req;

    req.can_id =
      ((uint32_t)node_ids_[i] << 5)
      | 0x009
      | CAN_RTR_FLAG;

    req.can_dlc = 0;

    ::write(
      can_socket_,
      &req,
      sizeof(req)
    );

  }



  /*
   * COLLECT RESPONSES WITH 10ms TIMEOUT
   */

  std::vector<bool> received(
    node_ids_.size(),
    false
  );

  int received_count = 0;

  const int needed =
    static_cast<int>(node_ids_.size());

  auto deadline =
    std::chrono::steady_clock::now()
    + std::chrono::milliseconds(10);



  while (received_count < needed)
  {

    auto now =
      std::chrono::steady_clock::now();

    if (now >= deadline)
    {
      break;
    }

    long us_left =
      std::chrono::duration_cast<
        std::chrono::microseconds
      >(deadline - now).count();

    struct timeval tv;
    tv.tv_sec  = 0;
    tv.tv_usec = static_cast<suseconds_t>(us_left);

    fd_set read_fds;
    FD_ZERO(&read_fds);
    FD_SET(can_socket_, &read_fds);

    int ret = select(
      can_socket_ + 1,
      &read_fds,
      nullptr,
      nullptr,
      &tv
    );

    if (ret <= 0)
    {
      break;
    }

    struct can_frame frame;

    int nbytes = ::read(
      can_socket_,
      &frame,
      sizeof(frame)
    );

    if (nbytes < static_cast<int>(sizeof(frame)))
    {
      continue;
    }

    /*
     * STRIP RTR / EFF / ERR FLAGS BEFORE PARSING
     */

    uint32_t raw_id =
      frame.can_id
      & ~(CAN_RTR_FLAG | CAN_EFF_FLAG | CAN_ERR_FLAG);

    int cmd_id  = static_cast<int>(raw_id & 0x1F);
    int node_id = static_cast<int>(raw_id >> 5);

    if (cmd_id != 0x009)
    {
      continue;
    }

    for (size_t i = 0; i < node_ids_.size(); i++)
    {

      if (node_ids_[i] == node_id && !received[i])
      {

        float output_turns;

        memcpy(
          &output_turns,
          &frame.data[0],
          sizeof(float)
        );

        /*
         * OUTPUT ENCODER TURNS -> JOINT RADIANS
         * Encoder is on output shaft, no gear ratio needed.
         */

        position_states_[i] =
          static_cast<double>(
            output_turns * 2.0f * static_cast<float>(M_PI)
          );

        received[i] = true;
        received_count++;
        break;

      }
    }
  }

  /*
   * CLOSED-LOOP VERIFICATION — print encoder feedback every ~2 s (200 cycles at 100 Hz)
   */
  if (received_count > 0)
  {
    static int enc_log_ctr = 0;
    if (++enc_log_ctr % 200 == 0)
    {
      std::cout << "[ENC]";
      for (size_t k = 0; k < position_states_.size(); k++)
        std::cout << " J" << k << "=" << position_states_[k] << "rad";
      std::cout << std::endl;
    }
  }

  return hardware_interface::return_type::OK;
}



/*
 * WRITE
 */

hardware_interface::return_type
SKAIHardware::write(
  const rclcpp::Time & time,
  const rclcpp::Duration & period
)
{

  /*
   * STARTUP SEQUENCE — runs once on the first write(), regardless of send gate.
   * Puts ODrive into closed-loop so the arm holds its current physical position.
   */

  if (!startup_done_)
  {

    clear_errors();

    usleep(100000);

    set_closed_loop();

    usleep(100000);

    startup_done_ = true;
  }

  /*
   * SEND GATE — position commands are only sent after the user presses a
   * send button (publishes true to /can_send_enable).
   */

  if (!send_enabled_)
  {
    return hardware_interface::return_type::OK;
  }

  /*
   * First time send is enabled: seed commands from live encoder positions
   * so the arm holds its current pose instead of snapping to 0.
   */

  if (!commands_seeded_)
  {
    for (size_t i = 0; i < position_commands_.size(); i++)
      position_commands_[i] = position_states_[i];

    commands_seeded_ = true;
  }




  for (
    size_t i = 0;
    i < position_commands_.size();
    i++
  )
  {

    /*
     * ROS2 Joint radians
     * -> ODrive motor turns
     */

    float joint_radians =
      position_commands_[i];

    /*
     * ODrive uses output encoder for position control,
     * so command in output shaft turns directly.
     */

    float motor_turns =
      joint_radians / (2.0f * static_cast<float>(M_PI));



    /*
     * ODRIVE CAN SIMPLE
     * SET_INPUT_POS
     */

    struct can_frame frame;

    int command_id = 0x0C;

    frame.can_id =

      (node_ids_[i] << 5)

      | command_id;

    /*
     * FULL ODRIVE PACKET
     */

    frame.can_dlc = 8;

    float pos = motor_turns;

    int16_t vel_ff = 0;

    int16_t torque_ff = 0;



    memcpy(
      &frame.data[0],
      &pos,
      4
    );

    memcpy(
      &frame.data[4],
      &vel_ff,
      2
    );

    memcpy(
      &frame.data[6],
      &torque_ff,
      2
    );



    /*
     * SEND CAN FRAME
     */

    int bytes_sent = ::write(
      can_socket_,
      &frame,
      sizeof(frame)
    );



    /*
     * AVOID CONSOLE SPAM
     */

    if (bytes_sent < 0)
    {

      static int error_counter = 0;

      error_counter++;

      if (error_counter % 100 == 0)
      {

        std::cout
          << strerror(errno)
          << std::endl;
      }
    }
  }

  return hardware_interface::return_type::OK;
}

} // namespace skai_hardware



PLUGINLIB_EXPORT_CLASS(

  skai_hardware::SKAIHardware,

  hardware_interface::SystemInterface

)