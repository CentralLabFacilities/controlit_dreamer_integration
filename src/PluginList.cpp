#include <pluginlib/class_list_macros.h>
#include <controlit/ServoClock.hpp>

#include <controlit/servo_clock_library/ServoClockDreamer.hpp>
#include <controlit/dreamer/RobotInterfaceDreamer.hpp>

// Defined in /opt/ros/indigo/include/pluginlib/class_list_macros.h:
//
PLUGINLIB_EXPORT_CLASS(controlit::servo_clock_library::ServoClockDreamer, controlit::ServoClock);
PLUGINLIB_EXPORT_CLASS(controlit::dreamer::RobotInterfaceDreamer, controlit::RobotInterface);
