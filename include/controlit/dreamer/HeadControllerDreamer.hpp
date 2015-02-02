#ifndef __CONTROLIT_DREAMER_INTEGRATION_HEAD_CONTROLLER_DREAMER_HPP__
#define __CONTROLIT_DREAMER_INTEGRATION_HEAD_CONTROLLER_DREAMER_HPP__

#include <ros/ros.h>
// #include <std_msgs/Bool.h>
#include <controlit/addons/eigen/LinearAlgebra.hpp>
#include <controlit/addons/ros/RealTimePublisherHeader.hpp>
#include <std_msgs/Float64.h>

using controlit::addons::eigen::Vector;
using controlit::addons::eigen::Matrix;
using controlit::addons::eigen::Matrix3d;

namespace controlit {
namespace dreamer {

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
     * Callback methods for head joint commands
     */
    void lowerNeckPitchCallback(const boost::shared_ptr<std_msgs::Float64 const> & msgPtr);

    // Local variables for holding the current position and velocity state.
    Vector currPosition;
    Vector currVelocity;


    // ROS Subscribers
    ros::Subscriber lowerNeckPitchSubscriber;

    // The command
    Vector commandPos;
    Vector commandVel;

    // ROS publishers
    std::unique_ptr<controlit::addons::ros::RealtimePublisherHeader<sensor_msgs::JointState>>
        jointStatePublisher;
};

} // namespace dreamer
} // namespace controlit

#endif  // __CONTROLIT_DREAMER_INTEGRATION_HEAD_CONTROLLER_DREAMER_HPP__
