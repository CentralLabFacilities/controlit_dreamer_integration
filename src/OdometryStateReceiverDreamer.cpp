#include <controlit/robot_interface_library/OdometryStateReceiverDreamer.hpp>

#include <thread>  // for std::thread::sleep_for(...)

#include <unistd.h>

namespace controlit {
namespace dreamer {

// Uncomment one of the following lines to enable/disable detailed debug statements.
#define PRINT_DEBUG_STATEMENT(ss)
// #define PRINT_DEBUG_STATEMENT(ss) CONTROLIT_DEBUG << ss;

#define PRINT_DEBUG_STATEMENT_RT(ss)
// #define PRINT_DEBUG_STATEMENT_RT(ss) CONTROLIT_DEBUG_RT << ss;

#define PRINT_INFO_STATEMENT_RT_ALWAYS(ss) CONTROLIT_INFO_RT << ss;
// #define PRINT_INFO_STATEMENT_RT_ALWAYS(ss) std::cout << ss << std::endl;

OdometryStateReceiverDreamer::OdometryStateReceiverDreamer() :
    OdometryStateReceiver() // Call super-class' constructor
{
}

OdometryStateReceiverDreamer::~OdometryStateReceiverDreamer()
{
}

bool OdometryStateReceiverDreamer::init(ros::NodeHandle & nh, RTControlModel * model)
{
    // Initialize the parent class.
    bool initSuccess = OdometryStateReceiver::init(nh, model);

    // If the super-class fails to initialize, abort.
    if (!initSuccess) return false;

    return true;
}

bool OdometryStateReceiverDreamer::getOdometry(const ros::Time & currTime, controlit::RobotState & latestRobotState, bool block)
{
    // TODO: Verify Eigen::Quaterniond::Identiy() is equal to w = 1, x = y = z = 0
    Eigen::Quaterniond q = Eigen::Quaterniond::Identity();
    CONTROLIT_ERROR_RT << "Testing Quaterniond:"
             << " q: [" << q.w() << ", " << q.x() << ", " << q.y() << ", " << q.z() << "]";

    // Dreamer is fixed to the world.  Just set the base state equal to zero.
    if (!latestRobotState.setRobotBaseState(Vector3d::Zero(), Eigen::Quaterniond::Identity(), Vector::Zero(6)))
    {
        CONTROLIT_WARN_RT << "Failed to set robot base state, aborting this read operation.";
        return false;
    }

    return true;
}

// NOTE: Odometry data is used to set the 6 virtual degrees of freedom.
// void OdometryStateReceiverDreamer::odometryMessageCallback(nav_msgs::Odometry& odom)
// {
//     // Assuming odometry information in the correct frame...
//     Vector x(3);
//     x(0) = odom.pose.pose.position.x;
//     x(1) = odom.pose.pose.position.y;
//     x(2) = odom.pose.pose.position.z;

//     // Get the orientation
//     Eigen::Quaterniond q(odom.pose.pose.orientation.w,
//         odom.pose.pose.orientation.x,
//         odom.pose.pose.orientation.y,
//         odom.pose.pose.orientation.z);

//     // Get the velocity
//     Vector x_dot(6);
//     x_dot(0) = odom.twist.twist.linear.x;
//     x_dot(1) = odom.twist.twist.linear.y;
//     x_dot(2) = odom.twist.twist.linear.z;
//     x_dot(3) = odom.twist.twist.angular.x;
//     x_dot(4) = odom.twist.twist.angular.y;
//     x_dot(5) = odom.twist.twist.angular.z;

//     if (!controlit::addons::eigen::checkMagnitude(x) || !controlit::addons::eigen::checkMagnitude(q) || !controlit::addons::eigen::checkMagnitude(x_dot))
//     {
//         CONTROLIT_ERROR_RT << "Received invalid odometry data!\n"
//             << "  - x: " << x.transpose() << "\n"
//             << "  - q: [" << q.w() << ", " << q.x() << ", " << q.y() << ", " << q.z() << "]\n"
//             << "  - x_dot: " << x_dot.transpose();
//         assert(false);
//     }
//     else
//     {
//         // the writeFromNonRT can be used in RT, if you have the guarantee that
//         //  * no non-rt thread is calling the same function (we're not subscribing to ros callbacks)
//         //  * there is only one single rt thread
//         odometryDataBuffer.writeFromNonRT( OdometryData(x, q, x_dot) );
//         receivedOdometryState = true;
//     }
// }

} // namespace dreamer
} // namespace controlit
