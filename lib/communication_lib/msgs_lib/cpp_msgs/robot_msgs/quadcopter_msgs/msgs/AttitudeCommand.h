#pragma once
#include "std_msgs/msgs/Header.h"

namespace cpp_msg {

struct AttitudeCommand {

  Header header;

  float roll;
  float pitch;
  float yaw;
};

} // namespace cpp_msg
