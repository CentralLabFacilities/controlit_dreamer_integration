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
// #define PRINT_INFO_STATEMENT(ss)
#define PRINT_INFO_STATEMENT(ss) CONTROLIT_INFO << ss;

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

/*!
 * This is a global method that takes as input a pointer to a RobotInterfaceDreamer
 * object and calls initSM() on it.
 *
 * \param[in] rid A pointer to the RobotInterfaceDreamer class.
 * \return the return value of the call to RobotInterfaceDreamer->initSM().
 */
void * call_initSMMethod(void * rid)
{
    RobotInterfaceDreamer * robotInterface = static_cast<RobotInterfaceDreamer*>(rid);
    return robotInterface->initSM(nullptr);
}


RobotInterfaceDreamer::RobotInterfaceDreamer() :
    RobotInterface() // Call super-class' constructor
    //rtThreadState(RT_THREAD_UNDEF)
    //receivedRobotState(false),
    //rcvdJointState(false)
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
    bool initSuccess = RobotInterface::init(nh, model);

    // If the super-class fails to initialize, abort.
    if(!initSuccess)
    {
        CONTROLIT_ERROR_RT << "Super-class failed to initialize.";
        return false;
    }

    // Change scheduler of this thread to be RTAI
    rt_allow_nonroot_hrt();
    RT_TASK * normalTask = rt_task_init_schmod(nam2num("TSHM"), NON_REALTIME_PRIORITY, 0, 0, SCHED_FIFO, 0xF);
    if (!normalTask)
        throw std::runtime_error("rt_task_init_schmod failed for non-RT task");

    // Spawn a real-time thread to initialize the shared memory connection
    int rtThreadID = rt_thread_create((void*)call_initSMMethod,
                                  this, // parameters
                                  10000); // XXXX 10000 is stack size I think

    
    // Wait for the real-time thread to exit
    rt_task_delete(normalTask);
    PRINT_INFO_STATEMENT("Waiting for real-time thread to finish initializing the pointer to shared memory and exit...");
    rt_thread_join(rtThreadID);
//----------------------------------------------------------------------------

    //parse robot description
    // urdf::Model model;
    // model.initParam("/robot_description");
    //
    //pull the relevant data into global structures
    // std::map<std::string, boost::shared_ptr<urdf::Joint> > joints = model.joints_;
    // unsigned long index = 0;
    // for(std::map<std::string, boost::shared_ptr<urdf::Joint> >::iterator iter = joints.begin(); iter != joints.end(); iter++)
    // {
    //   boost::shared_ptr<urdf::Joint> joint = iter->second;
    //   jointNameMap[joint->name] = index;
    //   std::cerr << joint->name << "->" << index << std::endl;
    //   index++;
    // }
    // const std::vector<std::string> & names = model->get()->getRealJointNamesVector();

    // PRINT_INFO_STATEMENT("Number of DoFs: " << names.size());

    // See Example here:  https://bitbucket.org/Jraipxg/matec_control/src/1d812060db36d7656e05d2cf8b8e182eccb1fcc2/atlas_sim/src/sm_plugin.cpp?at=develop

    // Create a shared memory location called "cmd" and initialize a 
    // torque command message that will be used to hold the torque command data.
    // PRINT_INFO_STATEMENT("Advertising topic \"/cmd\".");
    // cmdPublisher.advertise("/cmd");

    // torqueCmdMsg.layout.dim.resize(names.size());
    // torqueCmdMsg.layout.dim[0].stride = names.size();
    // torqueCmdMsg.layout.dim[0].size = names.size();
    // torqueCmdMsg.layout.dim[1].stride = 1;
    // torqueCmdMsg.layout.dim[1].size = 1;
    // torqueCmdMsg.data.resize(names.size());

    // PRINT_INFO_STATEMENT("Advertising topic \"/rtt_tx topic\".");
    // rttTxPublisher.advertise("/rtt_tx");

    // if (!rttRxSubscriber.subscribe("/rtt_rx", boost::bind(&RobotInterfaceDreamer::rttCallback, this, _1)))
    // {
    //     CONTROLIT_ERROR << "Failed to subscribe to topic \"/rtt_rx\"!";
    //     return false;
    // }

    // if (!robotStateSubscriber.subscribe("/joint_states", boost::bind(&RobotInterfaceDreamer::jointStateCallback, this, _1)))
    // {
    //     CONTROLIT_ERROR << "Failed to subscribe to topic \"/joint_states\"!";
    //     return false;
    // }

    // smi.subscribeSerializedROS<std_msgs::Int64>("/rtt_rx", boost::bind(&RobotInterfaceDreamer::rttCallback, this, _1));
    // smi.subscribeSerializedROS<sensor_msgs::JointState>("/joint_state", boost::bind(&RobotInterfaceDreamer::jointStateCallback, this, _1));

    // PRINT_INFO_STATEMENT("Loading joint names from shared memory...");
    // loadSMJointNameToIndexMap();
    // PRINT_INFO_STATEMENT("Joint names loaded!");

    // // Initialize the RTT latency publisher
    // rttLatencyPublisher.reset(new controlit::addons::ros::RealtimePublisher<std_msgs::Float64>(nh, "diagnostics/rttLatency", 1));
    // if(rttLatencyPublisher->trylock())
    // {
    //     rttLatencyPublisher->msg_.data = 0;
    //     rttLatencyPublisher->unlockAndPublish();
    // }
    // else
    // {
    //     CONTROLIT_ERROR_RT << "Unable to initialize rttLatencyPublisher!";
    //     return false;
    // }

    // rttSeqNo = 0;
    // rcvdRttSeqNo = 0;

    // Create the object that receives odometry information
    // For now we hard-code it to receive this data from a ROS topic
    PRINT_INFO_STATEMENT("Creating and initializing the odometry state receiver...");
    odometryStateReceiver.reset(new OdometryStateReceiverDreamer());
    return odometryStateReceiver->init(nh, model);
}

// This is a helper method that is run by RTAI's real-time scheduler and helps initialize
// the pointer to the shared memory and the semaphores that protect the shared memory.
void * RobotInterfaceDreamer::initSM(void*)
{
    // Switch to use RTAI real-time scheduler
    RT_TASK * task = rt_task_init_schmod(nam2num("TSHMP"), 0, 0, 0, SCHED_FIFO, 0xF);
    rt_allow_nonroot_hrt();
    if (task == nullptr) 
    {
        CONTROLIT_ERROR_RT << "Call to rt_task_init_schmod failed for TSHMP";
        return nullptr;
    }

    // Access the shared memory created by the M3 Server.
    sharedMemoryPtr = (M3Sds *) rt_shm_alloc(nam2num(TORQUE_SHM), sizeof(M3Sds), USE_VMALLOC);
    if (sharedMemoryPtr) 
    {
        CONTROLIT_INFO << "Found shared memory.";
    }
    else 
    {
        CONTROLIT_ERROR << "Call to rt_shm_alloc failed for shared memory name \"" << TORQUE_SHM << "\"";
        return nullptr;
    }

    // Get the semaphores protecting the status and command shared memory registers.
    status_sem = (SEM *) rt_get_adr(nam2num(TORQUE_STATUS_SEM));
    if ( ! status_sem) 
    {
      CONTROLIT_ERROR << "Torque status semaphore \"" << TORQUE_STATUS_SEM << "\" not found";
      return nullptr;
    }
    
    command_sem = (SEM *) rt_get_adr(nam2num(TORQUE_CMD_SEM));
    if ( ! command_sem) 
    {
      CONTROLIT_ERROR << "Torque command semaphore \"" << TORQUE_CMD_SEM << "\" not found";
      return nullptr;
    }

    rt_task_delete(task);
    return nullptr;
}

// void RobotInterfaceDreamer::rttCallback(std_msgs::Int64 & msg)
// {
//     rttRxMsgMutex.lock();

//     rttRxMsg = msg;
//     rcvdRTTRxMsg = true;

//     rttRxMsgMutex.unlock();
// }

// void RobotInterfaceDreamer::jointStateCallback(sensor_msgs::JointState & msg)
// {
//     jointStateMutex.lock();

//     jointStateMsg = msg;
//     rcvdJointState = true;
    
//     jointStateMutex.unlock();
// }

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
    // Reset the timestamp within robot state to remember when the state was obtained.
    latestRobotState.resetTimestamp();

    ///////////////////////////////////////////////////////////////////////////////////
    // Save the latest joint state into variable shm_status
    rt_sem_wait(status_sem);
    memcpy(&shm_status, sharedMemoryPtr->status, sizeof(shm_status));
    rt_sem_signal(status_sem);

    // Temporary code to print everything received
    printSHMStatus();

    ///////////////////////////////////////////////////////////////////////////////////
    // Save position data
    latestRobotState.setJointPosition(0, DEG_TO_RAD(shm_status.torso.theta[0])); // torso pan
    latestRobotState.setJointPosition(1, DEG_TO_RAD(shm_status.torso.theta[1])); // torso lower pitch
    latestRobotState.setJointPosition(2, DEG_TO_RAD(shm_status.torso.theta[2])); // torso upper pitch

    latestRobotState.setJointPosition(3, DEG_TO_RAD(shm_status.left_arm.theta[0])); // left arm
    latestRobotState.setJointPosition(4, DEG_TO_RAD(shm_status.left_arm.theta[1]));
    latestRobotState.setJointPosition(5, DEG_TO_RAD(shm_status.left_arm.theta[2]));
    latestRobotState.setJointPosition(6, DEG_TO_RAD(shm_status.left_arm.theta[3]));
    latestRobotState.setJointPosition(7, DEG_TO_RAD(shm_status.left_arm.theta[4]));
    latestRobotState.setJointPosition(8, DEG_TO_RAD(shm_status.left_arm.theta[5]));
    latestRobotState.setJointPosition(9, DEG_TO_RAD(shm_status.left_arm.theta[6]));

    latestRobotState.setJointPosition(10, DEG_TO_RAD(shm_status.right_arm.theta[0])); // right arm
    latestRobotState.setJointPosition(11, DEG_TO_RAD(shm_status.right_arm.theta[1]));
    latestRobotState.setJointPosition(12, DEG_TO_RAD(shm_status.right_arm.theta[2]));
    latestRobotState.setJointPosition(13, DEG_TO_RAD(shm_status.right_arm.theta[3]));
    latestRobotState.setJointPosition(14, DEG_TO_RAD(shm_status.right_arm.theta[4]));
    latestRobotState.setJointPosition(15, DEG_TO_RAD(shm_status.right_arm.theta[5]));
    latestRobotState.setJointPosition(16, DEG_TO_RAD(shm_status.right_arm.theta[6]));

    latestRobotState.setJointPosition(17, DEG_TO_RAD(shm_status.head.theta[0])); // neck
    latestRobotState.setJointPosition(18, DEG_TO_RAD(shm_status.head.theta[1]));
    latestRobotState.setJointPosition(19, DEG_TO_RAD(shm_status.head.theta[2]));
    latestRobotState.setJointPosition(20, DEG_TO_RAD(shm_status.head.theta[3]));

    ///////////////////////////////////////////////////////////////////////////////////
    // Save velocity data
    latestRobotState.setJointVelocity(0, DEG_TO_RAD(shm_status.torso.thetadot[0])); // torso pan
    latestRobotState.setJointVelocity(1, DEG_TO_RAD(shm_status.torso.thetadot[1])); // torso lower pitch
    latestRobotState.setJointVelocity(2, DEG_TO_RAD(shm_status.torso.thetadot[2])); // torso upper pitch

    latestRobotState.setJointVelocity(3, DEG_TO_RAD(shm_status.left_arm.thetadot[0])); // left arm
    latestRobotState.setJointVelocity(4, DEG_TO_RAD(shm_status.left_arm.thetadot[1]));
    latestRobotState.setJointVelocity(5, DEG_TO_RAD(shm_status.left_arm.thetadot[2]));
    latestRobotState.setJointVelocity(6, DEG_TO_RAD(shm_status.left_arm.thetadot[3]));
    latestRobotState.setJointVelocity(7, DEG_TO_RAD(shm_status.left_arm.thetadot[4]));
    latestRobotState.setJointVelocity(8, DEG_TO_RAD(shm_status.left_arm.thetadot[5]));
    latestRobotState.setJointVelocity(9, DEG_TO_RAD(shm_status.left_arm.thetadot[6]));

    latestRobotState.setJointVelocity(10, DEG_TO_RAD(shm_status.right_arm.thetadot[0])); // right arm
    latestRobotState.setJointVelocity(11, DEG_TO_RAD(shm_status.right_arm.thetadot[1]));
    latestRobotState.setJointVelocity(12, DEG_TO_RAD(shm_status.right_arm.thetadot[2]));
    latestRobotState.setJointVelocity(13, DEG_TO_RAD(shm_status.right_arm.thetadot[3]));
    latestRobotState.setJointVelocity(14, DEG_TO_RAD(shm_status.right_arm.thetadot[4]));
    latestRobotState.setJointVelocity(15, DEG_TO_RAD(shm_status.right_arm.thetadot[5]));
    latestRobotState.setJointVelocity(16, DEG_TO_RAD(shm_status.right_arm.thetadot[6]));

    latestRobotState.setJointVelocity(17, DEG_TO_RAD(shm_status.head.thetadot[0])); // neck
    latestRobotState.setJointVelocity(18, DEG_TO_RAD(shm_status.head.thetadot[1]));
    latestRobotState.setJointVelocity(19, DEG_TO_RAD(shm_status.head.thetadot[2]));
    latestRobotState.setJointVelocity(20, DEG_TO_RAD(shm_status.head.thetadot[3]));

    ///////////////////////////////////////////////////////////////////////////////////
    // Save effort data
    latestRobotState.setJointEffort(0, 1.0e-3 * shm_status.torso.torque[0]); // torso pan
    latestRobotState.setJointEffort(1, 1.0e-3 * shm_status.torso.torque[1]); // torso lower pitch
    latestRobotState.setJointEffort(2, 1.0e-3 * shm_status.torso.torque[2]); // torso upper pitch

    latestRobotState.setJointEffort(3, 1.0e-3 * shm_status.left_arm.torque[0]); // left arm
    latestRobotState.setJointEffort(4, 1.0e-3 * shm_status.left_arm.torque[1]);
    latestRobotState.setJointEffort(5, 1.0e-3 * shm_status.left_arm.torque[2]);
    latestRobotState.setJointEffort(6, 1.0e-3 * shm_status.left_arm.torque[3]);
    latestRobotState.setJointEffort(7, 1.0e-3 * shm_status.left_arm.torque[4]);
    latestRobotState.setJointEffort(8, 1.0e-3 * shm_status.left_arm.torque[5]);
    latestRobotState.setJointEffort(9, 1.0e-3 * shm_status.left_arm.torque[6]);

    latestRobotState.setJointEffort(10, 1.0e-3 * shm_status.right_arm.torque[0]); // right arm
    latestRobotState.setJointEffort(11, 1.0e-3 * shm_status.right_arm.torque[1]);
    latestRobotState.setJointEffort(12, 1.0e-3 * shm_status.right_arm.torque[2]);
    latestRobotState.setJointEffort(13, 1.0e-3 * shm_status.right_arm.torque[3]);
    latestRobotState.setJointEffort(14, 1.0e-3 * shm_status.right_arm.torque[4]);
    latestRobotState.setJointEffort(15, 1.0e-3 * shm_status.right_arm.torque[5]);
    latestRobotState.setJointEffort(16, 1.0e-3 * shm_status.right_arm.torque[6]);

    latestRobotState.setJointEffort(17, 1.0e-3 * shm_status.head.torque[0]); // neck
    latestRobotState.setJointEffort(18, 1.0e-3 * shm_status.head.torque[1]);
    latestRobotState.setJointEffort(19, 1.0e-3 * shm_status.head.torque[2]);
    latestRobotState.setJointEffort(20, 1.0e-3 * shm_status.head.torque[3]);

    ///////////////////////////////////////////////////////////////////////////////////
    // Get and save the latest odometry data
    if (!odometryStateReceiver->getOdometry(time, latestRobotState, block))
        return false;
    
    ///////////////////////////////////////////////////////////////////////////////////
    // Save the timestamp in the command message (not sure if this is necessary)
    shm_cmd.timestamp = shm_status.timestamp;

    return true;
}

bool RobotInterfaceDreamer::write(const ros::Time & time, const controlit::Command & command)
{
    if (command.getNumDOFs() != 21) return false;

    const Vector & cmd = command.getEffortCmd();

    ///////////////////////////////////////////////////////////////////////////////////
    // Save effort data into local variable
    shm_cmd.torso.tq_desired[0] = 1e3 * cmd[0]; // torso pan
    shm_cmd.torso.tq_desired[1] = 1e3 * cmd[1]; // torso lower pitch
    shm_cmd.torso.tq_desired[2] = 1e3 * cmd[2]; // torso upper pitch

    shm_cmd.left_arm.tq_desired[0] = 1e3 * cmd[3]; // left arm
    shm_cmd.left_arm.tq_desired[1] = 1e3 * cmd[4];
    shm_cmd.left_arm.tq_desired[2] = 1e3 * cmd[5];
    shm_cmd.left_arm.tq_desired[3] = 1e3 * cmd[6];
    shm_cmd.left_arm.tq_desired[4] = 1e3 * cmd[7];
    shm_cmd.left_arm.tq_desired[5] = 1e3 * cmd[8];
    shm_cmd.left_arm.tq_desired[6] = 1e3 * cmd[9];

    shm_cmd.right_arm.tq_desired[0] = 1e3 * cmd[10]; // right arm
    shm_cmd.right_arm.tq_desired[1] = 1e3 * cmd[11];
    shm_cmd.right_arm.tq_desired[2] = 1e3 * cmd[12];
    shm_cmd.right_arm.tq_desired[3] = 1e3 * cmd[13];
    shm_cmd.right_arm.tq_desired[4] = 1e3 * cmd[14];
    shm_cmd.right_arm.tq_desired[5] = 1e3 * cmd[15];
    shm_cmd.right_arm.tq_desired[6] = 1e3 * cmd[16];

    shm_cmd.head.tq_desired[0] = 1e3 * cmd[17]; // neck
    shm_cmd.head.tq_desired[1] = 1e3 * cmd[18];
    shm_cmd.head.tq_desired[2] = 1e3 * cmd[19];
    shm_cmd.head.tq_desired[3] = 1e3 * cmd[20];

    ///////////////////////////////////////////////////////////////////////////////////
    // Write commands to shared memory
    rt_sem_wait(command_sem);
    memcpy(sharedMemoryPtr->cmd, &shm_cmd, sizeof(shm_cmd));      
    rt_sem_signal(command_sem);

    return true;
}

} // namespace dreamer
} // namespace controlit
