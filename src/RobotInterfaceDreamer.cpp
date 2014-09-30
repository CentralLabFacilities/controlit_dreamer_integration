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

RobotInterfaceDreamer::RobotInterfaceDreamer() :
    RobotInterface() // Call super-class' constructor
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

    // Access the shared memory created by the M3 Server.
    sharedMemoryPtr = (M3Sds *) rt_shm_alloc(nam2num(TORQUE_SHM), sizeof(M3Sds), USE_VMALLOC);
    if (sharedMemoryPtr) 
    {
        CONTROLIT_ERROR << "Found shared memory.";
    }
    else 
    {
        CONTROLIT_ERROR << "rt_shm_alloc failed for " << TORQUE_SHM;
        return false;
    }

    // Get the semaphores protecting the status and command shared memory registers.
    status_sem = (SEM *) rt_get_adr(nam2num(TORQUE_STATUS_SEM));
    if ( ! status_sem) {
      fprintf(stderr, "semaphore %s not found\n", TORQUE_STATUS_SEM);
      return false;
    }
    
    command_sem = (SEM *) rt_get_adr(nam2num(TORQUE_CMD_SEM));
    if ( ! command_sem) {
      fprintf(stderr, "semaphore %s not found\n", TORQUE_CMD_SEM);
      return false;
    }

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
       << " - timestamp: " << shm_status.timestamp << "\n";
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

    ss << "M3TorqueShmSdsBaseCommand:\n";
       << " - timestamp: " << shm_cmd.timestamp << "\n";
       << " - right_arm:\n";

    printLimbSHMCommand(ss, "    ", shm_cmd.right_arm);

    ss << " - left_arm:\n";
    printLimbSHMCommand(ss, "    ", shm_cmd.left_arm);

    ss << " - torso:\n";
    printLimbSHMCommand(ss, "    ", shm_cmd.torso);

    ss << " - head:\n";
    printLimbSHMCommand(ss, "    ", shm_cmd.head);

    ss << " - right_hand:\n";
    printLimbSHMCommand(ss, "    ", shm_cmd.head);

    // NOTE: omitting mobile_base for now

    CONTROLIT_INFO << ss.str();
}

bool RobotInterfaceDreamer::read(const ros::Time & time, controlit::RobotState & latestRobotState, bool block)
{
    bool result = true;

    // Reset the timestamp within robot state to remember when the state was obtained.
    latestRobotState.resetTimestamp();

    // Read the status and initialize the 
    rt_sem_wait(status_sem);
    memcpy(&shm_status, sharedMemory->status, sizeof(shm_status));
    rt_sem_signal(status_sem);

    // Temporary code to print everything received
    
    for (size_t ii(0); ii < 7; ++ii) { // XXXX to do: hardcoded NDOF
      state.position_[ii] = M_PI * shm_status.right_arm.theta[ii] / 180.0;
      state.velocity_[ii] = M_PI * shm_status.right_arm.thetadot[ii] / 180.0;
      //state.force_[ii] = 1.0e-3 * shm_status.right_arm.torque[ii];
    }
    
    ///Force-Torque Sensor
    for (size_t jj(0); jj < 6; ++jj) {
      state.force_[jj] = 1.0e-3 * shm_status.right_arm.wrench[jj];
    }   

    cb_status = rtutil->init(state);
    if (0 != cb_status) {
      fprintf(stderr, "init callback returned %d\n", cb_status);
      rt_thread_state = RT_THREAD_ERROR;
      goto cleanup_init_callback;
    }

    // Check if RTT can be computed
    // std::vector<double> current_rtt;
    // smi.getCurrentFloatingPointVector("rtt", current_rtt);
    // assert(current_rtt.size() == 2);

    // The receiver will save the previously-transmitted RTT sequence number into the RTT Rx buffer.
    // rttRxMsgMutex.lock();

    // if (rcvdRTTRxMsg)
    // {
    //     if(rttRxMsg.data == rttSeqNo)
    //     {
    //         rcvdRttSeqNo = rttSeqNo; // This will trigger the sender to increment the rttSeqNo.

    //         if(rttLatencyPublisher->trylock()) // Only publish the RTT latency if the lock can be obtained.
    //         {
    //             // Compute the RTT.
    //             std::chrono::nanoseconds timeSpan = duration_cast < std::chrono::nanoseconds > (high_resolution_clock::now() - lastRTTUpdate);

    //             // Save the RTT in a message and publish it
    //             rttLatencyPublisher->msg_.data = timeSpan.count() / 1e9;
    //             rttLatencyPublisher->unlockAndPublish();
    //         }
    //     }
    // }

    // rttRxMsgMutex.unlock();

    // std::vector<double> pos_vel_torque_measurements;
    // smi.waitForFloatingPointMatrix("pos_vel_torque_measurement", pos_vel_torque_measurements);

    // std::vector<double> positions = std::vector<double>(pos_vel_torque_measurements.begin() + latestRobotState.getNumJoints() * 0, pos_vel_torque_measurements.begin() + latestRobotState.getNumJoints() * 1);
    // std::vector<double> velocities = std::vector<double>(pos_vel_torque_measurements.begin() + latestRobotState.getNumJoints() * 1, pos_vel_torque_measurements.begin() + latestRobotState.getNumJoints() * 2);
    // std::vector<double> torques = std::vector<double>(pos_vel_torque_measurements.begin() + latestRobotState.getNumJoints() * 2, pos_vel_torque_measurements.begin() + latestRobotState.getNumJoints() * 3);

    // jointStateMutex.lock();
    // if (rcvdJointState)
    // {
    //     // Sanity check to make sure the received joint state information is of the correct length
    //     if(latestRobotState.getNumJoints() != jointStateMsg.name.size())
    //     {
    //         CONTROLIT_ERROR << "Received joint state message of incorrect size!\n"
    //             << "  - expected: " << latestRobotState.getNumJoints() << "\n"
    //             << "  - received: " << jointStateMsg.name.size();
    //         return false;
    //     }

    //     // Initialize jointMsgToModelIndexMap if it has not already been initialized.
    //     // The order within the jointStateMsg is determined by the sender.  We need to find
    //     // the mapping between this order and the order used by ControlIt!.
    //     // if (jointMsgToModelIndexMap.get() == nullptr)
    //     // {
    //     //     jointMsgToModelIndexMap.reset(new std::vector<int>());
    //     //     modelToJointMsgIndexMap.reset(new std::vector<int>());
    //     //     modelToJointMsgIndexMap.resize(latestRobotState.getNumJoint());

    //     //     for (unsigned int ii = 0; ii < latestRobotState.getNumJoint(); ii++)
    //     //     {
    //     //         int jointIndex = latestRobotState.getJointIndex(jointStateMsg.name[ii]);
    //     //         jointMsgToModelIndexMap->push_back(jointIndex);   
    //     //         modelToJointMsgIndexMap->at(jointIndex) = ii;
    //     //     }
    //     // }

    //     // Intialize the jointNameMap if it has not already been initalized.
    //     // This mapping stores each joint's index within the shared memory.
    //     if (jointNameMap.size() == 0)
    //     {
    //         for (unsigned int ii = 0; ii < jointStateMsg.name.size(); ii++)
    //         {
    //             jointNameMap[jointStateMsg.name[ii]] = ii;
    //         }
    //     }

        
    //     for(unsigned int ii = 0; ii < latestRobotState.getNumJoints(); ii++)
    //     {
    //         // std::string name = latestRobotState.getJointNames()[ii];

    //         // // Get the name of the current joint
    //         // // std::string name = latestRobotState.getJointNames()[ii];

    //         // // Get the joint's index in the controlit::RobotState object
    //         // // TODO: Ensure the joint's name is in the map!
    //         // // unsigned int jointStateMsgIndex = jointNameMap[name];

    //         // // Update the joint state
    //         // if(jointStateMsgIndex < jointNameMap.size())
    //         // {
    //         //     latestRobotState.setJointPosition(jointIndex, jointStateMsg.position[jointStateMsgIndex]);
    //         //     latestRobotState.setJointVelocity(jointIndex, jointStateMsg.velocity[jointStateMsgIndex]);
    //         //     latestRobotState.setJointEffort(jointIndex, jointStateMsg.effort[jointStateMsgIndex]);

    //         //     #if PRINT_RECEIVED_STATE
    //         //     ss << "Joint: " << latestRobotState.getJointNames()[ii] << ", "
    //         //        << "position: " << pojointStateMsg.positionsitions[jointStateMsgIndex] << ", "
    //         //        << "velocity: " << jointStateMsg.velocity[jointStateMsgIndex] << ", "
    //         //        << "torque: " << jointStateMsg.effort[jointStateMsgIndex] << std::endl;
    //         //     #endif
    //         // }

    //         // Get the name of the current joint
    //         std::string name = latestRobotState.getJointNames()[ii];

    //         unsigned int shmIndx;
    //         try {
    //             shmIndx = jointNameMap.at(name);
    //         } 
    //         catch(std::out_of_range exception)
    //         {
    //             CONTROLIT_ERROR_RT << "Unknown joint name: " << name;
    //             result = false;
    //         }
            
    //         if (result)
    //         {
    //             // Update the joint state
    //             latestRobotState.setJointPosition(ii, jointStateMsg.position[shmIndx]);
    //             latestRobotState.setJointVelocity(ii, jointStateMsg.velocity[shmIndx]);
    //             latestRobotState.setJointEffort(ii, jointStateMsg.effort[shmIndx]);

    //             // TEMPORARY!
    //             if (std::abs(jointStateMsg.position[shmIndx]) > 1e3)
    //             {
    //                 std::cerr << "RobotInterfaceDreamer: questionable position: shmIndx = " << shmIndx << ", pos = " << jointStateMsg.position[shmIndx];
    //                 assert(false);
    //             }
    //         }
    //     }


    //     {
    //         // Check for valid values
    //         bool isValid = true;
    //         for(unsigned int ii = 0; ii < jointStateMsg.name.size() && isValid; ii++)            
    //         {
    //             if (std::abs(jointStateMsg.position[ii] >= 1e3)) isValid = false;
    //             if (std::abs(jointStateMsg.velocity[ii] >= 1e3)) isValid = false;
    //             if (std::abs(jointStateMsg.effort[ii] >= 1e3)) isValid = false;
    //         }

    //         if (!isValid)
    //             CONTROLIT_ERROR_RT << "Received questionable robot state:" << std::endl << jointStateMsgToString();
    //     }

        
    //     #if PRINT_RECEIVED_STATE
    //     CONTROLIT_INFO_RT << "Received state: " << std::endl; << jointStateMsgToString();
    //     #endif
        
    // } else
    //     result = false;

    // jointStateMutex.unlock();

    if (!result) return false;

    // Get and save the latest odometry data
    if (!odometryStateReceiver->getOdometry(time, latestRobotState, block))
    {
        // CONTROLIT_WARN_RT << "Failed to obtain odometry data.";
        return false;
    }
    
    return result;
}

// Note: this should only be called with thread holding lock on jointStateMutex
// std::string RobotInterfaceDreamer::jointStateMsgToString()
// {
//     std::stringstream ss;

//     for(unsigned int ii = 0; ii < jointStateMsg.name.size(); ii++)
//     {
//         ss << "  Joint: " << jointStateMsg.name[ii] << ", position: " << jointStateMsg.position[ii]
//            << ", velocity: " << jointStateMsg.velocity[ii] << ", torque: " << jointStateMsg.effort[ii] << std::endl;
//     }
//     return ss.str();
// }
    

// bool RobotInterfaceDreamer::loadSMJointNameToIndexMap()
// {
//     std::vector<std::string> sm_names;
//     smi.getCurrentStringVector("joint_names", sm_names);

//     std::stringstream ss;
//     for(unsigned int ii = 0; ii < sm_names.size(); ii++)
//     {
//         jointNameMap[sm_names[ii]] = ii;
// //        ss << sm_names[ii] << "->" << ii;
// //        if (ii < sm_names.size() - 1) ss << "\n";
//     }
//     PRINT_INFO_STATEMENT("Shared Memory Joint Name to Index Map (size: " << jointNameMap.size() << "):\n" << ss.str());

//     assert(jointNameMap.size() > 0);

//     return true;
// }

bool RobotInterfaceDreamer::write(const ros::Time & time, const controlit::Command & command)
{
//     if (jointNameMap.size() == 0) return false;

//     bool result = false;

//     // if (position_cmds.size() != jointNameMap.size())
//     // {
//     //     PRINT_INFO_STATEMENT("Resizing command variables to be of size " << jointNameMap.size() << ".");
//     //     position_cmds.resize(jointNameMap.size());
//     //     velocity_cmds.resize(jointNameMap.size());
//     //     torque_cmds.resize(jointNameMap.size());
//     // }

//     // The command may have fewer joints than the shared memory joint name map because
//     // of unactuated joints.
//     // assert(torqueCmdMsg.data.size() >= command.getNumDOFs());

    
//     // The command only applies to actuated joints
//     const std::vector<std::string> & controlit_names = model->get()->getActuatedJointNamesVector();

//     // Save the command into the message
//     for(unsigned int ii = 0; ii < command.getNumDOFs(); ii++)
//     {
//         int shmIndx = jointNameMap[controlit_names[ii]]; //TODO: make sure the string is in the map!

//         // if(shmIndx >= (int)jointNameMap.size())
//         // {
// //                std::cerr << controlit_names[ii] << ":" << shmIndx << "," << jointNameMap.size() << std::endl;
//             // assert(shmIndx < (int)jointNameMap.size());
//         // }

//         // CONTROLIT_INFO << "Saving command, shmIndx = " << shmIndx;
//         // position_cmds[shmIndx] = command.getPositionCmd()[ii];
//         // velocity_cmds[shmIndx] = command.getVelocityCmd()[ii];
//         // torque_cmds[shmIndx] = command.getEffortCmd()[ii];
//         torqueCmdMsg.data[shmIndx] = command.getEffortCmd()[ii];

//         // #if PRINT_COMMAND
//         // // ss << "  Joint: " << controlit_names[ii] << ", position_cmd: " << position_cmds[ii]
//         // //    << ", velocity_cmd: " << velocity_cmds[ii] << ", torque_cmd: " << torque_cmds[ii] << std::endl;
        
//         // #endif
//     }

    // Print command (useful for debugging purposes)
    // {
    //     #if PRINT_COMMAND
    //     std::stringstream ss;
        
    //     for(unsigned int ii = 0; ii < command.getNumDOFs(); ii++)
    //     {
    //         int shmIndx = jointNameMap[controlit_names[ii]];
    //         ss << "  Joint: " << controlit_names[ii] << ", effort_cmd: " << torqueCmdMsg.data[shmIndx] << std::endl;
    //     }

    //     std::cerr << "Writing command: " << std::endl << ss.str();
    //     #endif
    // }

    // cmdPublisher.publish(torqueCmdMsg);

    // If the current RTT sequence number equals the received RTT sequence number, 
    // increment the current RTT sequence number and write it to shared memory.
    // if(rttSeqNo == rcvdRttSeqNo)
    // {
    //     rttSeqNo++;
    //     currentRTTMsg.data = rttSeqNo;
    //     rttTxPublisher.publish(currentRTTMsg);
    //     lastRTTUpdate = high_resolution_clock::now();
    // }
    return result;
}

} // namespace dreamer
} // namespace controlit
