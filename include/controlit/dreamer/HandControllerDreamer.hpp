#ifndef __CONTROLIT_DREAMER_INTEGRATION_HAND_CONTROLLER_DREAMER_HPP__
#define __CONTROLIT_DREAMER_INTEGRATION_HAND_CONTROLLER_DREAMER_HPP__

#include <ros/ros.h>
// #include <std_msgs/Float64.h>
// #include <std_msgs/Float64MultiArray.h>
#include <std_msgs/Bool.h>
#include <controlit/addons/eigen/LinearAlgebra.hpp>

using controlit::addons::eigen::Vector;
using controlit::addons::eigen::Matrix;
using controlit::addons::eigen::Matrix3d;

namespace controlit {
namespace dreamer {

/*!
 * Supplies odometry information based on shared memory.
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
     * The callback method for updating the right hand goal joint positions.
     */
    // void rightHandGoalPosCallback(const boost::shared_ptr<std_msgs::Float64MultiArray const> & msgPtr);

    /*!
     * The callback method for updating the left hand goal joint positions.
     */
    // void leftHandGoalPosCallback(const boost::shared_ptr<std_msgs::Float64 const> & msgPtr);

    bool powerGraspRight;
    bool powerGraspLeft;
    bool closingRightFingers;

    Vector currPosition;

    Vector currVelocity;

    Vector kp;

    Vector kd;

    Vector goalPosition;

    ros::Subscriber rightHandPowerGraspSubscriber;

    ros::Subscriber leftGripperPowerGraspSubscriber;
};

} // namespace dreamer
} // namespace controlit

#endif  // __CONTROLIT_DREAMER_INTEGRATION_HAND_CONTROLLER_DREAMER_HPP__
