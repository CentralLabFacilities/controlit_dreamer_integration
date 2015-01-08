#ifndef __CONTROLIT_DREAMER_INTEGRATION_HAND_CONTROLLER_DREAMER_HPP__
#define __CONTROLIT_DREAMER_INTEGRATION_HAND_CONTROLLER_DREAMER_HPP__

#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <controlit/addons/eigen/LinearAlgebra.hpp>

using controlit::addons::eigen::Vector;
using controlit::addons::eigen::Matrix;
using controlit::addons::eigen::Matrix3d;

namespace controlit {
namespace dreamer {

/*!
 * Implements controllers for both end effectors on Dreamer. This consists of a
 * right hand and a left gripper.
 */
class HandControllerDreamer
{
public:
    /*!
     * The constructor.
     */
    HandControllerDreamer();

    /*!
     * The destructor.
     */
    ~HandControllerDreamer();

    /*!
     * Initializes this class.
     *
     * \param[in] nh The ROS node handle to use during initialization.
     * \return Whether the initialization was successful.
     */
    bool init(ros::NodeHandle & nh);

    /*!
     * Updates the state of the hand joints.
     *
     * \param[in] position The current position of the hand joints.
     * The joint order is:
     *
     *   0: right_thumb_cmc
     *   1: right_thumb_mcp
     *   2: right_pointer_finger_mcp
     *   3: right_middle_finger_mcp
     *   4: right_pinky_mcp
     *   5: left_gripper
     * 
     * \param[in] velocity The current velocity of the hand joints.
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
    void rightHandCallback(const boost::shared_ptr<std_msgs::Bool const> & msgPtr);


    /*!
     * The callback method for the left gripper power grasp control.
     */
    void leftGripperCallback(const boost::shared_ptr<std_msgs::Bool const> & msgPtr);

    /*!
     * Callback methods for including individual fingers in the power grasp.
     */
    void includeRightPinkyFingerSubscriber(const boost::shared_ptr<std_msgs::Bool const> & msgPtr);
    void includeRightMiddleFingerSubscriber(const boost::shared_ptr<std_msgs::Bool const> & msgPtr);
    void includeRightPointerFingerSubscriber(const boost::shared_ptr<std_msgs::Bool const> & msgPtr);

    bool powerGraspRight;
    bool powerGraspLeft;
    bool closingRightFingers;
    bool includeRightPointerFinger; // whether to include the right pointer finger in the power grasp
    bool includeRightMiddleFinger;  // whether to include the right middle finger in the power grasp
    bool includeRightPinkyFinger;   // whether to include the right pinky finger in the power grasp

    // Current state information
    Vector currPosition;
    Vector currVelocity;

    // Controller parameters
    double thumbKp;
    double thumbKd;
    double thumbGoalPos;

    // ROS Subscribers
    ros::Subscriber rightHandPowerGraspSubscriber;
    ros::Subscriber leftGripperPowerGraspSubscriber;
    ros::Subscriber includeRightPinkyFingerSubscriber;
    ros::Subscriber includeRightMiddleFingerSubscriber;
    ros::Subscriber includeRightPointerFingerSubscriber;
};

} // namespace dreamer
} // namespace controlit

#endif  // __CONTROLIT_DREAMER_INTEGRATION_HAND_CONTROLLER_DREAMER_HPP__
