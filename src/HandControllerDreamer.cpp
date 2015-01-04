#include <controlit/dreamer/HandControllerDreamer.hpp>
#include <controlit/logging/Logging.hpp>

namespace controlit {
namespace dreamer {

#define NUM_RIGHT_HAND_DOFS 5
#define NUM_COMMAND_DOFS 6                   // Command includes both left and right hands
#define NUM_TORQUE_CONTROLLED_JOINTS 6

#define RIGHT_THUMB_CMC_INDEX 0
#define LEFT_GRIPPER_JOINT_INDEX 5

// #define MAX_STEP_SIZE 0.1 // 5.7 degrees
#define MAX_STEP_SIZE 0.05 // 2.35 degrees

HandControllerDreamer::HandControllerDreamer()
{
}

HandControllerDreamer::~HandControllerDreamer()
{
}

bool HandControllerDreamer::init(ros::NodeHandle & nh)
{
    currPosition.setZero(NUM_COMMAND_DOFS);
    currVelocity.setZero(NUM_COMMAND_DOFS);
    goalPosition.setZero(NUM_COMMAND_DOFS);
    kp.setOnes(NUM_TORQUE_CONTROLLED_JOINTS);
    kd.setZero(NUM_TORQUE_CONTROLLED_JOINTS);

    kp[0] = 1.5;

    // Subscribe to hand goal messages
    nh.subscribe("controlit/rightHand/goalPosition", 1, 
        & HandControllerDreamer::rightHandGoalPosCallback, this);

    nh.subscribe("controlit/leftHand/goalPosition", 1, 
        & HandControllerDreamer::leftHandGoalPosCallback, this);

    return true;
}

void HandControllerDreamer::updateState(Vector position, Vector velocity)
{
    currPosition = position;
    currVelocity = velocity;
}

void HandControllerDreamer::getCommand(Vector & command)
{
    assert(command.size() == NUM_COMMAND_DOFS);
    
    // Index 0 contains the right thumb position command.
    // command[RIGHT_THUMB_CMC_INDEX] = goalPosition[RIGHT_THUMB_CMC_INDEX];

    // Indices 1-4 contain the finger torque commands.
    // Index 5 contains the gripper torque command.

    for (size_t ii = 0; ii < NUM_COMMAND_DOFS; ii++) 
    {
       // create a linear trajectory
       double currGoal = goalPosition[ii];

       if (goalPosition[ii] > currPosition[ii])
       {
           if (goalPosition[ii] - currPosition[ii] > MAX_STEP_SIZE)
             currGoal = currPosition[ii] + MAX_STEP_SIZE;   
       }
       else
       {
           if (currPosition[ii] - goalPosition[ii] > MAX_STEP_SIZE)
               currGoal = currPosition[ii] - MAX_STEP_SIZE;
       }

       // compute the PD control law
       command[ii] = kp[ii] * (currGoal - currPosition[ii]) - kd[ii] * currVelocity[ii];

       if (ii > 0)
           command[ii] = 0;  // DEBUG 
    }
}

void HandControllerDreamer::rightHandGoalPosCallback(const boost::shared_ptr<std_msgs::Float64MultiArray const> & msgPtr)
{
    if (msgPtr->layout.dim[0].size != NUM_RIGHT_HAND_DOFS)
    {
        CONTROLIT_ERROR << "Invalid right hand goal position message. Contained " << msgPtr->layout.dim[0].size << " values, expected " << NUM_RIGHT_HAND_DOFS;
    }
    else
    {
        for (int ii = 0; ii < NUM_RIGHT_HAND_DOFS; ii++)
        {
            goalPosition[ii] = msgPtr->data[ii];
        }
    }
}

void HandControllerDreamer::leftHandGoalPosCallback(const boost::shared_ptr<std_msgs::Float64 const> & msgPtr)
{
    goalPosition[LEFT_GRIPPER_JOINT_INDEX] = msgPtr->data;
}

} // namespace dreamer
} // namespace controlit
