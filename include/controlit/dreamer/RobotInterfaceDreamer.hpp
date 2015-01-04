#ifndef __CONTROLIT_DREAMER_INTEGRATION_ROBOT_INTERFACE_DREAMER_HPP__
#define __CONTROLIT_DREAMER_INTEGRATION_ROBOT_INTERFACE_DREAMER_HPP__

#include <controlit/addons/ros/RealTimePublisher.hpp>
#include <controlit/RobotInterface.hpp>
#include <controlit/dreamer/HandControllerDreamer.hpp>

// #include <thread>  // for std::mutex
#include <unistd.h>
#include "m3uta/controllers/torque_shm_uta_sds.h"
#include <urdf/model.h>

#include <rtai_sem.h> // for SEM

using std::chrono::high_resolution_clock;
using std::chrono::duration_cast;

namespace controlit {
namespace dreamer {

/*!
 * A robot interface to Dreamer hardware.  This communicates with Dreamer via
 * shared memory created by the M3 server.
 */
class RobotInterfaceDreamer : public controlit::RobotInterface
{
public:
    /*!
     * The constructor.
     */
    RobotInterfaceDreamer();

    /*!
     * The destructor.
     */
    virtual ~RobotInterfaceDreamer();

    /*!
     * Initializes this robot interface.
     *
     * \param[in] nh The ROS node handle to use during the initialization
     * process.
     * \param[in] model The robot model.
     * \return Whether the initialization was successful.
     */
    virtual bool init(ros::NodeHandle & nh, RTControlModel * model);

    /*!
     * Obtains the current state of the robot.
     *
     * \param[out] latestRobotState The variable in which to store the
     * latest robot state.
     * \param[in] block Whether to block waiting for message to arrive.
     * \return Whether the read was successful.
     */
    virtual bool read(controlit::RobotState & latestRobotState, bool block = false);

    /*!
     * Sends a command to the robot.
     *
     * \param[in] command The command to send to the robot.
     * \return Whether the write was successful.
     */
    virtual bool write(const controlit::Command & command);

    /*!
     * Gets the current time in seconds.
     *
     * \return The current time in seconds.
     */
    virtual double getTime();

private:

    /*!
     * This is executed by the real-time thread. It initializes the connection to shared memory.
     */
    bool initSM();

    void printLimbSHMStatus(std::stringstream & ss, std::string prefix, 
        M3TorqueShmSdsBaseStatus & shmLimbStatus);

    void printSHMStatus();

    void printLimbSHMCommand(std::stringstream & ss, std::string prefix, 
        M3TorqueShmSdsBaseCommand & shmLimbCommand);

    void printSHMCommand();

    /*!
     * Whether the shared memory variables are initialized.
     */
    bool sharedMemoryReady;

    /*!
     * Maps the name of a joint to its index within the shared memory.
     */
    std::map<std::string, int> jointNameMap;

    /*!
     * A pointer to the shared memory used to communicate with the M3
     * server.  M3Sds stands for "M3 Shared Data Structure".
     */
    M3Sds * sharedMemoryPtr;

    /*!
     * A pointer to the semaphore protecting the status register.
     */
    SEM * status_sem;

    /*!
     * A pointer to the semaphore protecting the command register.
     */
    SEM * command_sem;

    /*!
     * Holds a copy of the status that was read from the shared memory.
     * It is defined in mekabot/m3uta/src/m3uta/controllers/torque_shm_uta_sds.h.
     */
    M3UTATorqueShmSdsStatus shm_status;

    /*!
     * Holds a copy of the command to be written to shared memory.
     */
    M3UTATorqueShmSdsCommand shm_cmd;

    /*!
     * The object that generates the commands for the hands.
     */
    HandControllerDreamer handController;

    /*!
     * The current hand joint positions.
     */
    Vector handJointPositions;

    /*!
     * The current hand joint velocities.
     */
    Vector handJointVelocities;

    /*!
     * The command to send to the hand.
     */
    Vector handCommand;
};

} // namespace dreamer
} // namespace controlit

#endif // __CONTROLIT_DREAMER_INTEGRATION_ROBOT_INTERFACE_DREAMER_HPP__
