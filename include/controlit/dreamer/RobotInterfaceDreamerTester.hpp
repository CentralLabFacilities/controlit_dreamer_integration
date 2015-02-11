#ifndef __CONTROLIT_ROBOT_INTERFACE_DREAMER_TESTER_HPP__
#define __CONTROLIT_ROBOT_INTERFACE_DREAMER_TESTER_HPP__

#include "ros/ros.h"
#include <controlit/dreamer/ServoClockDreamer.hpp>
#include <controlit/dreamer/TimerRTAI.hpp>
#include <controlit/addons/ros/RealTimePublisher.hpp>
#include <controlit/ServoableClass.hpp>
#include <controlit/dreamer/RobotInterfaceDreamer.hpp>
#include <controlit/RTControlModel.hpp>
#include <controlit/utility/ControlItParameters.hpp>
#include <controlit/BindingManager.hpp>

#include <std_msgs/Float64.h>

namespace controlit {
namespace dreamer {

/*!
 * A class that provides the main method for launching a ControlIt!
 * controller.
 */
class RobotInterfaceDreamerTester : controlit::ServoableClass
{
public:
    /*!
     * The default constructor.
     */
    RobotInterfaceDreamerTester();

    /*!
     * The destructor.
     */
    virtual ~RobotInterfaceDreamerTester();

    /*!
     * Initializes this tester.
     *
     * \return Whether the initialization was successful.
     */
    bool init();

    /*!
     * Starts this tester.
     *
     * \param[in] freq The desired servo frequency in Hz.
     */
    bool start(double freq);

    /*!
     * Stops this tester.
     */
    bool stop();

    /*!
     * Called once the first time the servo clock is started.
     */
    void servoInit();

    /*!
     * The callback function for the Servo Clock.
     */
    void servoUpdate();

private:

    /*!
     * A ROS node handle.  Used during the initialization of the robot interface.
     */
    ros::NodeHandle nh;

    /*!
     * The model that's used during the initialization of the robot interface.
     */
    controlit::RTControlModel model;

    /*!
     * For holding the robot state that's read from the robot interface.
     */
    controlit::RobotState robotState;

    /*!
     * The servo clock. This is used to call RobotInterfaceDreamer's read method.
     */
    ServoClockDreamer servoClock;

    /*!
     * The robot interface being tested.
     */
    RobotInterfaceDreamer robotInterface;

    /*!
     * The timer used to measure the servo period.
     */
    TimerRTAI timer;

    /*!
     * A real-time safe publisher for publishing the servo period.
     */
    controlit::addons::ros::RealtimePublisher<std_msgs::Float64> publisher;

    controlit::BindingManager bindingManager;

    controlit::utility::ControlItParameters params;
};

} // namespace dreamer
} // namespace controlit

#endif  // __CONTROLIT_ROBOT_INTERFACE_DREAMER_TESTER_HPP__
