#include "skai_hardware/odrive_driver.hpp"

#include <cstring>
#include <cstdint>
#include <cmath>

#include <sys/ioctl.h>
#include <sys/socket.h>
#include <unistd.h>
#include <sys/select.h>

namespace skai_hardware
{

ODriveDriver::ODriveDriver(
  const std::string & can_interface
)
: can_interface_(can_interface),
  can_socket_(-1)
{
}

ODriveDriver::~ODriveDriver()
{
  if (can_socket_ >= 0)
  {
    close(can_socket_);
  }
}

int ODriveDriver::init()
{
  can_socket_ = socket(
    PF_CAN,
    SOCK_RAW,
    CAN_RAW
  );

  if (can_socket_ < 0)
  {
    return -1;
  }

  strcpy(
    ifr_.ifr_name,
    can_interface_.c_str()
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
    return -1;
  }

  return 0;
}

void ODriveDriver::clearErrors(
  const std::vector<int>& node_ids
)
{
  for (int node_id : node_ids)
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
}

void ODriveDriver::setClosedLoop(
  const std::vector<int>& node_ids
)
{
  for (int node_id : node_ids)
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
}

bool ODriveDriver::readPosition(
  int node_id,
  double & position_rad
)
{
  struct can_frame req;

  req.can_id =
    ((uint32_t)node_id << 5)
    | 0x009
    | CAN_RTR_FLAG;

  req.can_dlc = 0;

  ::write(
    can_socket_,
    &req,
    sizeof(req)
  );

  struct timeval tv;
  tv.tv_sec = 0;
  tv.tv_usec = 2000;

  fd_set readfds;
  FD_ZERO(&readfds);
  FD_SET(can_socket_, &readfds);

  int ret = select(
    can_socket_ + 1,
    &readfds,
    nullptr,
    nullptr,
    &tv
  );

  if (ret <= 0)
  {
    return false;
  }

  struct can_frame frame;

  int nbytes =
    ::read(
      can_socket_,
      &frame,
      sizeof(frame)
    );

  if (nbytes < (int)sizeof(frame))
  {
    return false;
  }

  float output_turns;

  memcpy(
    &output_turns,
    &frame.data[0],
    sizeof(float)
  );

  position_rad =
    output_turns *
    2.0 *
    M_PI;

  return true;
}

bool ODriveDriver::setPosition(
  int node_id,
  double position_rad
)
{
  float turns =
    position_rad /
    (2.0f * static_cast<float>(M_PI));

  struct can_frame frame;

  frame.can_id =
    (node_id << 5)
    | 0x0C;

  frame.can_dlc = 8;

  float pos = turns;
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

  return (
    ::write(
      can_socket_,
      &frame,
      sizeof(frame)
    ) > 0
  );
}

}
