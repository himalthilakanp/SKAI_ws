#pragma once

#include <vector>
#include <string>

#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>

namespace skai_hardware
{

class ODriveDriver
{
public:

  explicit ODriveDriver(
    const std::string & can_interface = "can0"
  );

  ~ODriveDriver();

  int init();

  void clearErrors(
    const std::vector<int>& node_ids
  );

  void setClosedLoop(
    const std::vector<int>& node_ids
  );

  bool readPosition(
    int node_id,
    double & position_rad
  );

  bool setPosition(
    int node_id,
    double position_rad
  );

private:

  int can_socket_;

  struct sockaddr_can addr_;

  struct ifreq ifr_;

  std::string can_interface_;
};

}