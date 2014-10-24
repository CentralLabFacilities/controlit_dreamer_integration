#include <controlit/dreamer/RobotInterfaceDreamer.hpp>

#include <chrono>
#include <controlit/Command.hpp>
#include <controlit/RTControlModel.hpp>
#include <controlit/logging/RealTimeLogging.hpp>
#include <controlit/dreamer/OdometryStateReceiverDreamer.hpp>

#include "m3/robots/chain_name.h"
#include <m3rt/base/m3ec_def.h>
#include <m3rt/base/m3rt_def.h>

#include <rtai_shm.h>

#define TORQUE_SHM "TSHMM"
#define TORQUE_CMD_SEM "TSHMC"
#define TORQUE_STATUS_SEM "TSHMS"

namespace controlit {
namespace dreamer {

// Uncomment one of the following lines to enable/disable detailed debug statements.
#define PRINT_INFO_STATEMENT(ss)
// #define PRINT_INFO_STATEMENT(ss) CONTROLIT_INFO << ss;

#define PRINT_INFO_STATEMENT_RT(ss)
// #define PRINT_INFO_STATEMENT_RT(ss) CONTROLIT_INFO_RT << ss;

#define PRINT_INFO_RT(ss)
// #define PRINT_INFO_RT(ss) CONTROLIT_INFO_RT << ss;

#define PRINT_INFO_STATEMENT_RT_ALWAYS(ss) CONTROLIT_INFO_RT << ss;
// #define PRINT_INFO_STATEMENT_RT_ALWAYS(ss) std::cout << ss << std::endl;

#define PRINT_RECEIVED_STATE 0
#define PRINT_COMMAND 0

#define NON_REALTIME_PRIORITY 1

#define DEG_TO_RAD(deg) deg / 180 * 3.14159265359

RobotInterfaceDreamer::RobotInterfaceDreamer() :
    RobotInterface(),         // Call super-class' constructor
    sharedMemoryReady(false)
{
}

RobotInterfaceDreamer::~RobotInterfaceDreamer()
{
}

bool RobotInterfaceDreamer::init(ros::NodeHandle & nh, RTControlModel * model)
{
    PRINT_INFO_STATEMENT("Method called!");

    // Wait until robot_description becomes available on ROS parameter server.
    while(!nh.hasParam("/robot_description") && ros::ok())
    {
        ROS_WARN_THROTTLE(2.0, "RobotInterfaceDreamer is waiting for robot description.");
    }

    // Initialize the parent class.
    if (!RobotInterface::init(nh, model))
        return false;

    // Create the odometry receiver.
    PRINT_INFO_STATEMENT("Creating and initializing the odometry state receiver...");
    odometryStateReceiver.reset(new OdometryStateReceiverDreamer());
    return odometryStateReceiver->init(nh, model);
}

// This needs to be called by the RT thread provided by ServoClockDreamer.
// It is called the first time either read() or write() is called.
bool RobotInterfaceDreamer::initSM()
{
    PRINT_INFO_STATEMENT("Method called!");

    // Get a pointer to the shared memory created by the M3 Server.
    PRINT_INFO_STATEMENT("Getting point to shared memory...");
    sharedMemoryPtr = (M3Sds *) rt_shm_alloc(nam2num(TORQUE_SHM), sizeof(M3Sds), USE_VMALLOC);
    if (!sharedMemoryPtr)
    {
        CONTROLIT_ERROR << "Call to rt_shm_alloc failed for shared memory name \"" << TORQUE_SHM << "\"";
        return false;
    }

    // Get the semaphores protecting the status and command shared memory registers.
    PRINT_INFO_STATEMENT("Getting shared memory semaphores...");
    status_sem = (SEM *) rt_get_adr(nam2num(TORQUE_STATUS_SEM));
    if (!status_sem) 
    {
      CONTROLIT_ERROR << "Torque status semaphore \"" << TORQUE_STATUS_SEM << "\" not found";
      return false;
    }
    
    command_sem = (SEM *) rt_get_adr(nam2num(TORQUE_CMD_SEM));
    if (!command_sem) 
    {
      CONTROLIT_ERROR << "Torque command semaphore \"" << TORQUE_CMD_SEM << "\" not found";
      return false;
    }

    PRINT_INFO_STATEMENT("Done initializing connection to shared memory.");
    sharedMemoryReady = true;  // Prevents this method from being called again.

    return true;
}

void RobotInterfaceDreamer::printLimbSHMStatus(std::stringstream & ss, std::string prefix, 
    M3TorqueShmSdsBaseStatus & shmLimbStatus)
{
    ss << prefix << " - theta: [";
    for (size_t ii = 0; ii < MAX_NDOF; ii++)
    {
        ss << shmLimbStatus.theta[ii];
        if (ii < MAX_NDOF - 1) ss << ", ";
    }
    ss << "]\n";

    ss << prefix << " - thetadot: [";
    for (size_t ii = 0; ii < MAX_NDOF; ii++)
    {
        ss << shmLimbStatus.thetadot[ii];
        if (ii < MAX_NDOF - 1) ss << ", ";
    }
    ss << "]\n";

    ss << prefix << " - torque: [";
    for (size_t ii = 0; ii < MAX_NDOF; ii++)
    {
        ss << shmLimbStatus.torque[ii];
        if (ii < MAX_NDOF - 1) ss << ", ";
    }
    ss << "]\n";

    ss << prefix << " - wrench: [";
    for (size_t ii = 0; ii < 6; ii++)
    {
        ss << shmLimbStatus.wrench[ii];
        if (ii < 6 - 1) ss << ", ";
    }
    ss << "]\n";

    ss << prefix << " - ctrl_mode: [";
    for (size_t ii = 0; ii < MAX_NDOF; ii++)
    {
        ss << shmLimbStatus.ctrl_mode[ii];
        if (ii < MAX_NDOF - 1) ss << ", ";
    }
    ss << "]\n";

}

void RobotInterfaceDreamer::printSHMStatus()
{
    std::stringstream ss;

    ss << "M3UTATorqueShmSdsStatus:\n"
       << " - timestamp: " << shm_status.timestamp << "\n"
       << " - right_arm:\n";

    printLimbSHMStatus(ss, "    ", shm_status.right_arm);

    ss << " - left_arm:\n";
    printLimbSHMStatus(ss, "    ", shm_status.left_arm);

    ss << " - torso:\n";
    printLimbSHMStatus(ss, "    ", shm_status.torso);

    ss << " - head:\n";
    printLimbSHMStatus(ss, "    ", shm_status.head);

    ss << " - right_hand:\n";
    printLimbSHMStatus(ss, "    ", shm_status.head);

    // NOTE: omitting mobile_base for now

    CONTROLIT_INFO << ss.str();
}

void RobotInterfaceDreamer::printLimbSHMCommand(std::stringstream & ss, std::string prefix, 
    M3TorqueShmSdsBaseCommand & shmLimbCommand)
{
    ss << prefix << " - tq_desired: [";
    for (size_t ii = 0; ii < MAX_NDOF; ii++)
    {
        ss << shmLimbCommand.tq_desired[ii];
        if (ii < MAX_NDOF - 1) ss << ", ";
    }
    ss << "]\n";

    ss << prefix << " - q_desired: [";
    for (size_t ii = 0; ii < MAX_NDOF; ii++)
    {
        ss << shmLimbCommand.q_desired[ii];
        if (ii < MAX_NDOF - 1) ss << ", ";
    }
    ss << "]\n";

    ss << prefix << " - slew_rate_q_desired: [";
    for (size_t ii = 0; ii < MAX_NDOF; ii++)
    {
        ss << shmLimbCommand.slew_rate_q_desired[ii];
        if (ii < MAX_NDOF - 1) ss << ", ";
    }
    ss << "]\n";

    ss << prefix << " - q_stiffness: [";
    for (size_t ii = 0; ii < 6; ii++)
    {
        ss << shmLimbCommand.q_stiffness[ii];
        if (ii < 6 - 1) ss << ", ";
    }
    ss << "]\n";
}

void RobotInterfaceDreamer::printSHMCommand()
{
    std::stringstream ss;

    ss << "M3TorqueShmSdsBaseCommand:\n"
       << " - timestamp: " << shm_cmd.timestamp << "\n"
       << " - right_arm:\n";

    printLimbSHMCommand(ss, "    ", shm_cmd.right_arm);

    ss << " - left_arm:\n";
    printLimbSHMCommand(ss, "    ", shm_cmd.left_arm);

    ss << " - torso:\n";
    printLimbSHMCommand(ss, "    ", shm_cmd.torso);

    ss << " - head:\n";
    printLimbSHMCommand(ss, "    ", shm_cmd.head);

    ss << " - right_hand:\n";
    printLimbSHMCommand(ss, "    ", shm_cmd.right_hand);

    // NOTE: omitting mobile_base for now

    CONTROLIT_INFO << ss.str();
}

bool RobotInterfaceDreamer::read(const ros::Time & time, controlit::RobotState & latestRobotState, bool block)
{
    PRINT_INFO_STATEMENT("Method called!");

    if (!sharedMemoryReady)
    {
        if (!initSM()) return false;
    }

    // Reset the timestamp within robot state to remember when the state was obtained.
    latestRobotState.resetTimestamp();

    ///////////////////////////////////////////////////////////////////////////////////
    // Save the latest joint state into variable shm_status
    PRINT_INFO_STATEMENT("Grabbing lock on status semaphore...");
    rt_sem_wait(status_sem);
    memcpy(&shm_status, sharedMemoryPtr->status, sizeof(shm_status));
    rt_sem_signal(status_sem);
    PRINT_INFO_STATEMENT("Releasing lock on status semaphore...");

    // Temporary code to print everything received
    // printSHMStatus();

    ///////////////////////////////////////////////////////////////////////////////////
    // Save position data
    // latestRobotState.setJointPosition(0, DEG_TO_RAD(shm_status.torso.theta[0])); // torso pan
    // latestRobotState.setJointPosition(0, 0); // torso pan, fixed to zero since joint is not working as of 2014/10/16
    latestRobotState.setJointPosition(0, DEG_TO_RAD(shm_status.torso.theta[1])); // torso_pitch_1
    latestRobotState.setJointPosition(1, DEG_TO_RAD(shm_status.torso.theta[2])); // torso_pitch_2

    latestRobotState.setJointPosition(2, DEG_TO_RAD(shm_status.left_arm.theta[0])); // left arm
    latestRobotState.setJointPosition(3, DEG_TO_RAD(shm_status.left_arm.theta[1]));
    latestRobotState.setJointPosition(4, DEG_TO_RAD(shm_status.left_arm.theta[2]));
    latestRobotState.setJointPosition(5, DEG_TO_RAD(shm_status.left_arm.theta[3]));
    latestRobotState.setJointPosition(6, DEG_TO_RAD(shm_status.left_arm.theta[4]));
    latestRobotState.setJointPosition(7, DEG_TO_RAD(shm_status.left_arm.theta[5]));
    latestRobotState.setJointPosition(8, DEG_TO_RAD(shm_status.left_arm.theta[6]));

    latestRobotState.setJointPosition(9, DEG_TO_RAD(shm_status.head.theta[0])); // neck
    latestRobotState.setJointPosition(10, DEG_TO_RAD(shm_status.head.theta[1]));
    latestRobotState.setJointPosition(11, DEG_TO_RAD(shm_status.head.theta[2]));
    latestRobotState.setJointPosition(12, DEG_TO_RAD(shm_status.head.theta[3]));

    latestRobotState.setJointPosition(13, DEG_TO_RAD(shm_status.right_arm.theta[0])); // right arm
    latestRobotState.setJointPosition(14, DEG_TO_RAD(shm_status.right_arm.theta[1]));
    latestRobotState.setJointPosition(15, DEG_TO_RAD(shm_status.right_arm.theta[2]));
    latestRobotState.setJointPosition(16, DEG_TO_RAD(shm_status.right_arm.theta[3]));
    latestRobotState.setJointPosition(17, DEG_TO_RAD(shm_status.right_arm.theta[4]));
    latestRobotState.setJointPosition(18, DEG_TO_RAD(shm_status.right_arm.theta[5]));
    latestRobotState.setJointPosition(19, DEG_TO_RAD(shm_status.right_arm.theta[6]));



    ///////////////////////////////////////////////////////////////////////////////////
    // Save velocity data
    // latestRobotState.setJointVelocity(0, DEG_TO_RAD(shm_status.torso.thetadot[0])); // torso pan
    // latestRobotState.setJointVelocity(0, 0); // torso pan, fixed to zero since joint is not working as of 2014/10/16
    latestRobotState.setJointVelocity(0, DEG_TO_RAD(shm_status.torso.thetadot[1])); // torso_pitch_1
    latestRobotState.setJointVelocity(1, DEG_TO_RAD(shm_status.torso.thetadot[2])); // torso_pitch_2

    latestRobotState.setJointVelocity(2, DEG_TO_RAD(shm_status.left_arm.thetadot[0])); // left arm
    latestRobotState.setJointVelocity(3, DEG_TO_RAD(shm_status.left_arm.thetadot[1]));
    latestRobotState.setJointVelocity(4, DEG_TO_RAD(shm_status.left_arm.thetadot[2]));
    latestRobotState.setJointVelocity(5, DEG_TO_RAD(shm_status.left_arm.thetadot[3]));
    latestRobotState.setJointVelocity(6, DEG_TO_RAD(shm_status.left_arm.thetadot[4]));
    latestRobotState.setJointVelocity(7, DEG_TO_RAD(shm_status.left_arm.thetadot[5]));
    latestRobotState.setJointVelocity(8, DEG_TO_RAD(shm_status.left_arm.thetadot[6]));

    latestRobotState.setJointVelocity(9, DEG_TO_RAD(shm_status.head.thetadot[0])); // neck
    latestRobotState.setJointVelocity(10, DEG_TO_RAD(shm_status.head.thetadot[1]));
    latestRobotState.setJointVelocity(11, DEG_TO_RAD(shm_status.head.thetadot[2]));
    latestRobotState.setJointVelocity(12, DEG_TO_RAD(shm_status.head.thetadot[3]));

    latestRobotState.setJointVelocity(13, DEG_TO_RAD(shm_status.right_arm.thetadot[0])); // right arm
    latestRobotState.setJointVelocity(14, DEG_TO_RAD(shm_status.right_arm.thetadot[1]));
    latestRobotState.setJointVelocity(15, DEG_TO_RAD(shm_status.right_arm.thetadot[2]));
    latestRobotState.setJointVelocity(16, DEG_TO_RAD(shm_status.right_arm.thetadot[3]));
    latestRobotState.setJointVelocity(17, DEG_TO_RAD(shm_status.right_arm.thetadot[4]));
    latestRobotState.setJointVelocity(18, DEG_TO_RAD(shm_status.right_arm.thetadot[5]));
    latestRobotState.setJointVelocity(19, DEG_TO_RAD(shm_status.right_arm.thetadot[6]));

    ///////////////////////////////////////////////////////////////////////////////////
    // Save effort data
    // latestRobotState.setJointEffort(0, 1.0e-3 * shm_status.torso.torque[0]); // torso pan
    // latestRobotState.setJointEffort(0, 0); // torso pan, fixed to zero since joint is not working as of 2014/10/16
    latestRobotState.setJointEffort(0, 1.0e-3 * shm_status.torso.torque[1]); // torso_pitch_1
    latestRobotState.setJointEffort(1, 1.0e-3 * shm_status.torso.torque[2]); // torso_pitch_2

    latestRobotState.setJointEffort(2, 1.0e-3 * shm_status.left_arm.torque[0]); // left arm
    latestRobotState.setJointEffort(3, 1.0e-3 * shm_status.left_arm.torque[1]);
    latestRobotState.setJointEffort(4, 1.0e-3 * shm_status.left_arm.torque[2]);
    latestRobotState.setJointEffort(5, 1.0e-3 * shm_status.left_arm.torque[3]);
    latestRobotState.setJointEffort(6, 1.0e-3 * shm_status.left_arm.torque[4]);
    latestRobotState.setJointEffort(7, 1.0e-3 * shm_status.left_arm.torque[5]);
    latestRobotState.setJointEffort(8, 1.0e-3 * shm_status.left_arm.torque[6]);

    latestRobotState.setJointEffort(9, 1.0e-3 * shm_status.head.torque[0]); // neck
    latestRobotState.setJointEffort(10, 1.0e-3 * shm_status.head.torque[1]);
    latestRobotState.setJointEffort(11, 1.0e-3 * shm_status.head.torque[2]);
    latestRobotState.setJointEffort(12, 1.0e-3 * shm_status.head.torque[3]);

    latestRobotState.setJointEffort(13, 1.0e-3 * shm_status.right_arm.torque[0]); // right arm
    latestRobotState.setJointEffort(14, 1.0e-3 * shm_status.right_arm.torque[1]);
    latestRobotState.setJointEffort(15, 1.0e-3 * shm_status.right_arm.torque[2]);
    latestRobotState.setJointEffort(16, 1.0e-3 * shm_status.right_arm.torque[3]);
    latestRobotState.setJointEffort(17, 1.0e-3 * shm_status.right_arm.torque[4]);
    latestRobotState.setJointEffort(18, 1.0e-3 * shm_status.right_arm.torque[5]);
    latestRobotState.setJointEffort(19, 1.0e-3 * shm_status.right_arm.torque[6]);



    ///////////////////////////////////////////////////////////////////////////////////
    // Get and save the latest odometry data
    if (!odometryStateReceiver->getOdometry(time, latestRobotState, block))
        return false;

    return true;
}

bool RobotInterfaceDreamer::write(const ros::Time & time, const controlit::Command & command)
{
    PRINT_INFO_STATEMENT("Method called!");

    if (command.getNumDOFs() != 21) return false;
 
    if (!sharedMemoryReady)
    {
        if (!initSM()) return false;
    }

    const Vector & cmd = command.getEffortCmd();

    ///////////////////////////////////////////////////////////////////////////////////
    // Save effort data into local variable
    // shm_cmd.torso.tq_desired[0] = 1e3 * cmd[0]; // torso pan
    shm_cmd.torso.tq_desired[0] = 0; // torso_yaw, fixed to zero since joint is not working as of 2014/10/16
    shm_cmd.torso.tq_desired[1] = 1e3 * cmd[0]; // torso_pitch_1
    shm_cmd.torso.tq_desired[2] = 1e3 * cmd[1]; // torso_pitch_2

    shm_cmd.left_arm.tq_desired[0] = 1e3 * cmd[2]; // left arm
    shm_cmd.left_arm.tq_desired[1] = 1e3 * cmd[3];
    shm_cmd.left_arm.tq_desired[2] = 1e3 * cmd[4];
    shm_cmd.left_arm.tq_desired[3] = 1e3 * cmd[5];
    shm_cmd.left_arm.tq_desired[4] = 1e3 * cmd[6];
    shm_cmd.left_arm.tq_desired[5] = 1e3 * cmd[7];
    shm_cmd.left_arm.tq_desired[6] = 1e3 * cmd[8];

    shm_cmd.head.tq_desired[0] = 1e3 * cmd[9]; // neck
    shm_cmd.head.tq_desired[1] = 1e3 * cmd[10];
    shm_cmd.head.tq_desired[2] = 1e3 * cmd[11];
    shm_cmd.head.tq_desired[3] = 1e3 * cmd[12];

    shm_cmd.right_arm.tq_desired[0] = 1e3 * cmd[13]; // right arm
    shm_cmd.right_arm.tq_desired[1] = 1e3 * cmd[14];
    shm_cmd.right_arm.tq_desired[2] = 1e3 * cmd[15];
    shm_cmd.right_arm.tq_desired[3] = 1e3 * cmd[16];
    shm_cmd.right_arm.tq_desired[4] = 1e3 * cmd[17];
    shm_cmd.right_arm.tq_desired[5] = 1e3 * cmd[18];
    shm_cmd.right_arm.tq_desired[6] = 1e3 * cmd[19];

    ///////////////////////////////////////////////////////////////////////////////////
    // Save the timestamp in the command message (not sure if this is necessary)
    shm_cmd.timestamp = shm_status.timestamp;

    CONTROLIT_INFO << "Setting command time stamp to be " << shm_cmd.timestamp;

    ///////////////////////////////////////////////////////////////////////////////////
    // Write commands to shared memory
    PRINT_INFO_STATEMENT("Getting lock on command semaphore...");
    rt_sem_wait(command_sem);
    memcpy(sharedMemoryPtr->cmd, &shm_cmd, sizeof(shm_cmd));      
    rt_sem_signal(command_sem);
    PRINT_INFO_STATEMENT("Releasing lock on command semaphore...");

    return true;
}

} // namespace dreamer
} // namespace controlit
