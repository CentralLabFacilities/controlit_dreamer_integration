#ifndef __CONTROLIT_DREAMER_INTEGRATION_HEAD_CONTROLLER_DREAMER_HPP__
#define __CONTROLIT_DREAMER_INTEGRATION_HEAD_CONTROLLER_DREAMER_HPP__

#include <ros/ros.h>
// #include <std_msgs/Bool.h>
#include <controlit/addons/eigen/LinearAlgebra.hpp>
#include <controlit/addons/ros/RealTimePublisherHeader.hpp>
#include <std_msgs/Float64MultiArray.h>

#include <SerialStream.h>

using controlit::addons::eigen::Vector;
using controlit::addons::eigen::Matrix;
using controlit::addons::eigen::Matrix3d;

namespace controlit {
namespace dreamer {

#define SERIAL_BUFFER_SIZE 18
#define SERIAL_START_BYTE 0x55

/*!
 * Implements controllers for Dreamer's head joints.
 */
class HeadControllerDreamer
{
public:
    /*!
     * The constructor.
     */
    HeadControllerDreamer();

    /*!
     * The destructor.
     */
    ~HeadControllerDreamer();

    /*!
     * Initializes this class.
     *
     * \param[in] nh The ROS node handle to use during initialization.
     * \return Whether the initialization was successful.
     */
    bool init(ros::NodeHandle & nh);

    /*!
     * Updates the state of the head joints.
     *
     * \param[in] position The current position of the head joints.
     * The joint order is:
     *
     *   0: lower_neck_pitch
     *   1: upper_neck_yaw
     *   2: upper_neck_roll
     *   3: upper_neck_pitch
     *   4: eye_pitch
     *   5: right_eye_yaw
     *   6: left_eye_yaw
     *
     * \param[in] velocity The current velocity of the head joints.
     */
    void updateState(Vector position, Vector velocity);

    /*!
     * Obtains the command based on the current state and current goal
     * positions.
     */
    void getCommand(Vector & command);

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
private:

    /*!
     * Callback methods for head joint position commands.
     */
    void positionCommandCallback(const boost::shared_ptr<std_msgs::Float64MultiArray const> & msgPtr);

    /*!
     * Callback method for head joint position error values.
     */
    void positionErrorCallback(const boost::shared_ptr<std_msgs::Float64MultiArray const> & msgPtr);

    // Local variables for holding the current position and velocity state.
    Vector currPosition;
    Vector currVelocity;

    // ROS Subscribers
    ros::Subscriber headPositionCommandSubscriber;
    ros::Subscriber headPositionErrorSubscriber;

    // The command
    Vector commandPos;
    // Vector commandVel;
    Vector errorPos;

    // ROS publishers
    controlit::addons::ros::RealtimePublisherHeader<sensor_msgs::JointState>
        jointStatePublisher;

    controlit::addons::ros::RealtimePublisherHeader<sensor_msgs::JointState>
        jointCommandPublisher;

    LibSerial::SerialStream serialPort;

    char serialOutputBuff[SERIAL_BUFFER_SIZE];
};

} // namespace dreamer
} // namespace controlit

#endif  // __CONTROLIT_DREAMER_INTEGRATION_HEAD_CONTROLLER_DREAMER_HPP__
