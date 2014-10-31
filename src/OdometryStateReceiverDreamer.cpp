#include <controlit/dreamer/OdometryStateReceiverDreamer.hpp>

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

bool OdometryStateReceiverDreamer::getOdometry(controlit::RobotState & latestRobotState, bool block)
{
    // Dreamer is fixed to the world.  Just set the base state equal to zero.
    if (!latestRobotState.setRobotBaseState(Vector3d::Zero(), Eigen::Quaterniond::Identity(), Vector::Zero(6)))
    {
        CONTROLIT_WARN_RT << "Failed to set robot base state, aborting this read operation.";
        return false;
    }

    return true;
}

} // namespace dreamer
} // namespace controlit
