#ifndef __CONTROLIT_DREAMER_INTEGRATION_ODOMETRY_STATE_RECEIVER_DREAMER_HPP__
#define __CONTROLIT_DREAMER_INTEGRATION_ODOMETRY_STATE_RECEIVER_DREAMER_HPP__

#include <ros/ros.h>  // for ROS Time
#include <controlit/RobotState.hpp>
#include <controlit/OdometryData.hpp>

#include <nav_msgs/Odometry.h>

#include <controlit/OdometryStateReceiver.hpp>
#include <controlit/EigenRealtimeBuffer.hpp>

// #include <shared_memory_interface/shared_memory_subscriber.hpp>

namespace controlit {
namespace dreamer {

/*!
 * Supplies odometry information based on shared memory.
 */
class OdometryStateReceiverDreamer : public OdometryStateReceiver
{
public:
    /*!
     * The constructor.
     */
    OdometryStateReceiverDreamer();

    /*!
     * The destructor.
     */
    ~OdometryStateReceiverDreamer();

    /*!
     * Initializes this class.
     *
     * \param[in] nh The ROS node handle to use during initialization.
     * \param[in] model The robot model.
     * \return Whether the initialization was successful.
     */
    virtual bool init(ros::NodeHandle & nh, RTControlModel * model);

    /*!
     * Obtains the current odometry state.
     *
     * \param[out] latestRobotState This is where the robot's odometry state is stored.
     * \param[in] block Whether to block waiting for the odometry state to arrive.
     * \return Whether the odometry state was successfully obtained.
     */
    virtual bool getOdometry(controlit::RobotState & latestRobotState, bool block = false);

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
private:
};

} // namespace dreamer
} // namespace controlit

#endif  // __CONTROLIT_DREAMER_INTEGRATION_ODOMETRY_STATE_RECEIVER_DREAMER_HPP__
