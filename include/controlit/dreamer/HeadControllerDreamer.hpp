#ifndef __CONTROLIT_DREAMER_INTEGRATION_HEAD_CONTROLLER_DREAMER_HPP__
#define __CONTROLIT_DREAMER_INTEGRATION_HEAD_CONTROLLER_DREAMER_HPP__

#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <controlit/addons/eigen/LinearAlgebra.hpp>
#include <controlit/addons/ros/RealTimePublisherHeader.hpp>

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
     * The callback method for the right hand power grasp control.
     */
    // void rightHandCallback(const boost::shared_ptr<std_msgs::Bool const> & msgPtr);

    /*!
     * Callback methods for including individual fingers in the power grasp.
     */
    // void includeRightPinkyFingerCallback(const boost::shared_ptr<std_msgs::Bool const> & msgPtr);
    // void includeRightMiddleFingerCallback(const boost::shared_ptr<std_msgs::Bool const> & msgPtr);
    // void includeRightPointerFingerCallback(const boost::shared_ptr<std_msgs::Bool const> & msgPtr);

    // bool powerGraspRight;
    // bool powerGraspLeft;
    // bool closingRightFingers;
    // bool includeRightPointerFinger; // whether to include the right pointer finger in the power grasp
    // bool includeRightMiddleFinger;  // whether to include the right middle finger in the power grasp
    // bool includeRightPinkyFinger;   // whether to include the right pinky finger in the power grasp

    // Current state information
    // Vector currPosition;
    // Vector currVelocity;

    // Controller parameters
    // double thumbKp;
    // double thumbKd;
    // double thumbGoalPos;

    // ROS Subscribers
    // ros::Subscriber rightHandPowerGraspSubscriber;
    // ros::Subscriber leftGripperPowerGraspSubscriber;
    // ros::Subscriber includeRightPinkyFingerSubscriber;
    // ros::Subscriber includeRightMiddleFingerSubscriber;
    // ros::Subscriber includeRightPointerFingerSubscriber;

     std::unique_ptr<controlit::addons::ros::RealtimePublisherHeader<sensor_msgs::JointState>>
        jointStatePublisher;

};

} // namespace dreamer
} // namespace controlit

#endif  // __CONTROLIT_DREAMER_INTEGRATION_HEAD_CONTROLLER_DREAMER_HPP__
