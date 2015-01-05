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

#define POWER_GRASP_ENABLED_KP 2.25
#define POWER_GRASP_DISABLED_KP 3.0

HandControllerDreamer::HandControllerDreamer() :
    powerGraspRight(false),
    powerGraspLeft(false)
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

    // Set the Kp for the right_thumb_cmc joint
    kp[0] = POWER_GRASP_DISABLED_KP;
     
     // Subscribe to hand goal messages
    // nh.subscribe("controlit/rightHand/goalPosition", 1, 
    //     & HandControllerDreamer::rightHandGoalPosCallback, this);

    // nh.subscribe("controlit/leftHand/goalPosition", 1, 
    //     & HandControllerDreamer::leftHandGoalPosCallback, this);

    // CONTROLIT_INFO << "Subscribing to power grasp topic...";
    rightHandPowerGraspSubscriber = nh.subscribe("controlit/rightHand/powerGrasp", 1, 
        & HandControllerDreamer::rightHandCallback, this);

    leftGripperPowerGraspSubscriber = nh.subscribe("controlit/leftGripper/powerGrasp", 1,
        & HandControllerDreamer::leftGripperCallback, this);

    // CONTROLIT_INFO << "Subscribed to topic: " << rightHandPowerGraspSubscriber.getTopic();
    return true;
}

void HandControllerDreamer::updateState(Vector position, Vector velocity)
{
    currPosition = position;
    currVelocity = velocity;
}

void HandControllerDreamer::getCommand(Vector & command)
{
    // assert(command.size() == NUM_COMMAND_DOFS);

    command.setZero(6); // for debugging, reset everything to zero 

    if (powerGraspRight)
    {
        kp[0] = POWER_GRASP_ENABLED_KP;
        goalPosition[0] = 0; // right_thumb_cmc 90 degrees from palm
        
        // wait until right_thumb_cmc is at the zero position before curling the fingers
        if (std::abs(currPosition[0]) < 0.17)  // 0.17 radians is 10 degrees
        {
            command[4] = command[3] = command[2] = command[1] = 0.3;  // right_thumb_mcp, right_pointer_finger_mcp, right_pointer_finger_mcp, right_pinky_mcp
        }
        else
        {
            // CONTROLIT_INFO << "Not curling right_thumb_mcp, current position of right_thumb_cmc is " << std::abs(currPosition[0]);
        }
    }
    else
    {
        command[4] = command[3] = command[2] = command[1] = -0.1;  // right_thumb_mcp, right_pointer_finger_mcp, right_pointer_finger_mcp, right_pinky_mcp
        
        // wait until all fingers are relaxed before moving the right_thumb_cmc
        if (std::abs(currPosition[1]) < 0.02)
        {
            kp[0] = POWER_GRASP_DISABLED_KP;
            goalPosition[0] = 1.57;  // right_thumb_cmc 180 degrees from palm
        }
        else
        {
            // CONTROLIT_INFO << "Not putting right_thumb_cmc into relax position, current positions of fingers:\n"
            //               << "  - right_thumb_mcp: " << std::abs(currPosition[1]);
        }
    }
    
    // Index 0 contains the right thumb position command.
    // command[RIGHT_THUMB_CMC_INDEX] = goalPosition[RIGHT_THUMB_CMC_INDEX];

    // Indices 1-4 contain the finger torque commands.
    // Index 5 contains the gripper torque command.

    // for (size_t ii = 0; ii < NUM_COMMAND_DOFS; ii++) 
    // {
       // create a linear trajectory
       
    // Do position control of right_thumb_cmc
    double currGoal = goalPosition[0];

    if (goalPosition[0] > currPosition[0])
    {
        if (goalPosition[0] - currPosition[0] > MAX_STEP_SIZE)
            currGoal = currPosition[0] + MAX_STEP_SIZE;   
    }
    else
    {
        if (currPosition[0] - goalPosition[0] > MAX_STEP_SIZE)
            currGoal = currPosition[0] - MAX_STEP_SIZE;
    }

    // compute the PD control law for right_thumb_cmc
    command[0] = kp[0] * (currGoal - currPosition[0]) - kd[0] * currVelocity[0];

    // Issue command to left gripper
    command[5] = powerGraspLeft ? 2 : -0.5;
}

void HandControllerDreamer::rightHandCallback(const boost::shared_ptr<std_msgs::Bool const> & msgPtr)
{
    powerGraspRight = msgPtr->data;
    // CONTROLIT_INFO << "Right hand power grasp: " << (powerGraspRight ? "TRUE" : "FALSE");
}

void HandControllerDreamer::leftGripperCallback(const boost::shared_ptr<std_msgs::Bool const> & msgPtr)
{
    powerGraspLeft = msgPtr->data;
    CONTROLIT_INFO << "Left gripper power grasp: " << (powerGraspLeft ? "TRUE" : "FALSE");
}

// void HandControllerDreamer::rightHandGoalPosCallback(const boost::shared_ptr<std_msgs::Float64MultiArray const> & msgPtr)
// {
//     if (msgPtr->layout.dim[0].size != NUM_RIGHT_HAND_DOFS)
//     {
//         CONTROLIT_ERROR << "Invalid right hand goal position message. Contained " << msgPtr->layout.dim[0].size << " values, expected " << NUM_RIGHT_HAND_DOFS;
//     }
//     else
//     {
//         for (int ii = 0; ii < NUM_RIGHT_HAND_DOFS; ii++)
//         {
//             goalPosition[ii] = msgPtr->data[ii];
//         }
//     }
// }

// void HandControllerDreamer::leftHandGoalPosCallback(const boost::shared_ptr<std_msgs::Float64 const> & msgPtr)
// {
//     goalPosition[LEFT_GRIPPER_JOINT_INDEX] = msgPtr->data;
// }

} // namespace dreamer
} // namespace controlit
