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
// #define MAX_STEP_SIZE 0.05 // 2.35 degrees
#define MAX_STEP_SIZE 0.01 // 0.57 degrees

#define POWER_GRASP_ENABLED_KP 3.5
// #define POWER_GRASP_DISABLED_KP 5.0

HandControllerDreamer::HandControllerDreamer() :
    powerGraspRight(false),
    powerGraspLeft(false),
    closingRightFingers(false),
    includeRightPointerFinger(true),
    includeRightMiddleFinger(true),
    includeRightPinkyFinger(true),
    thumbKp(POWER_GRASP_ENABLED_KP),
    thumbKd(0),
    thumbGoalPos(0),
    rhCommandPublisher("controlit/rightHand/command", 1),
    rhStatePublisher("controlit/rightHand/state", 1)
{
}

HandControllerDreamer::~HandControllerDreamer()
{
}

bool HandControllerDreamer::init(ros::NodeHandle & nh)
{
    currPosition.setZero(NUM_COMMAND_DOFS);
    currVelocity.setZero(NUM_COMMAND_DOFS);

    // CONTROLIT_INFO << "Subscribing to power grasp topic...";
    rightHandPowerGraspSubscriber = nh.subscribe("controlit/rightHand/powerGrasp", 1, 
        & HandControllerDreamer::rightHandCallback, this);

    includeRightPinkyFingerSubscriber = nh.subscribe("controlit/rightHand/includeRightPinkyFinger", 1, 
        & HandControllerDreamer::includeRightPinkyFingerCallback, this);

    includeRightMiddleFingerSubscriber = nh.subscribe("controlit/rightHand/includeRightMiddleFinger", 1, 
        & HandControllerDreamer::includeRightMiddleFingerCallback, this);

    includeRightPointerFingerSubscriber = nh.subscribe("controlit/rightHand/includeRightIndexFinger", 1, 
        & HandControllerDreamer::includeRightPointerFingerCallback, this);

    leftGripperPowerGraspSubscriber = nh.subscribe("controlit/leftGripper/powerGrasp", 1,
        & HandControllerDreamer::leftGripperCallback, this);

    //---------------------------------------------------------------------------------
    // Initialize the publishers of the latest right hand state and command.
    //---------------------------------------------------------------------------------
    if(rhStatePublisher.trylock())
    {
        rhStatePublisher.msg_.name.push_back("right_thumb_cmc");
        rhStatePublisher.msg_.name.push_back("right_thumb_mcp");
        rhStatePublisher.msg_.name.push_back("right_pointer_finger_mcp");
        rhStatePublisher.msg_.name.push_back("right_middle_finger_mcp");
        rhStatePublisher.msg_.name.push_back("right_pinky_mcp");


        for (size_t ii = 0; ii < NUM_RIGHT_HAND_DOFS; ii++)
        {   
            rhStatePublisher.msg_.position.push_back(0.0);  // allocate memory for the joint states
            rhStatePublisher.msg_.velocity.push_back(0.0);
            rhStatePublisher.msg_.effort.push_back(0.0);
        }

        rhStatePublisher.unlockAndPublish();
    }
    else
    {
        CONTROLIT_ERROR << "Unable to initialize the right hand state publisher!";
        return false;
    }

    if(rhCommandPublisher.trylock())
    {
        rhCommandPublisher.msg_.name.push_back("right_thumb_cmc");
        rhCommandPublisher.msg_.name.push_back("right_thumb_mcp");
        rhCommandPublisher.msg_.name.push_back("right_pointer_finger_mcp");
        rhCommandPublisher.msg_.name.push_back("right_middle_finger_mcp");
        rhCommandPublisher.msg_.name.push_back("right_pinky_mcp");


        for (size_t ii = 0; ii < NUM_RIGHT_HAND_DOFS; ii++)
        {
            rhCommandPublisher.msg_.position.push_back(0.0);  // allocate memory for the joint states
            rhCommandPublisher.msg_.velocity.push_back(0.0);
            rhCommandPublisher.msg_.effort.push_back(0.0);
        }
        rhCommandPublisher.unlockAndPublish();
    }
    else
    {
        CONTROLIT_ERROR << "Unable to initialize the right hand command publisher!";
        return false;
    }

    return true;
}

void HandControllerDreamer::updateState(Vector position, Vector velocity)
{
    currPosition = position;
    currVelocity = velocity;

    if(rhStatePublisher.trylock())
    {
        for (size_t ii = 0; ii < NUM_RIGHT_HAND_DOFS; ii++)
        {   
            rhStatePublisher.msg_.position[ii] =  currPosition[ii];
            rhStatePublisher.msg_.velocity[ii] = currVelocity[ii];
        }

        rhStatePublisher.unlockAndPublish();
    }
}

void HandControllerDreamer::getCommand(Vector & command)
{
    command.setZero(6); // for debugging, reset everything to zero 

    if (powerGraspRight)
    {
        if (!closingRightFingers)
            thumbKp = POWER_GRASP_ENABLED_KP;
        else
            thumbKp = 1; //POWER_GRASP_DISABLED_KP;
        thumbGoalPos = 0; // right_thumb_cmc 90 degrees from palm
        
        // wait until right_thumb_cmc is at the zero position before curling the fingers
        if (std::abs(currPosition[0]) < 0.2)  // 0.2 radians is 11.46 degrees
        {
            closingRightFingers = true;
            thumbKp = 1; //POWER_GRASP_DISABLED_KP;

            command[1] = 0.3; // close right_thumb_mcp
            command[2] = (includeRightPointerFinger ? 0.3 : -0.1);  // right_pointer_finger
            command[3] = (includeRightMiddleFinger  ? 0.3 : -0.1);  // right_middle_finger
            command[4] = (includeRightPinkyFinger   ? 0.3 : -0.1);  // right_pinky_finger
        }
        else
        {
            // CONTROLIT_INFO << "Not curling right_thumb_mcp, current position of right_thumb_cmc is " << std::abs(currPosition[0]);
        }
    }
    else
    {
        closingRightFingers = false;
        command[4] = command[3] = command[2] = command[1] = -0.1;  // right_thumb_mcp, right_pointer_finger_mcp, right_pointer_finger_mcp, right_pinky_mcp
        
        // wait until all fingers are relaxed before moving the right_thumb_cmc
        if (std::abs(currPosition[1]) < 0.02)
        {
            // thumbKp = POWER_GRASP_DISABLED_KP;
            thumbGoalPos = 1.57;  // right_thumb_cmc at 90 degree position
            // thumbGoalPos = 1.396; // right_thumb_cmc at 80 degree position
            command[0] = 0.1; // Issue 0.1 Nm of torque to force the joint to go to position 1.57 radians
        }
        else
        {
            // CONTROLIT_INFO << "Not putting right_thumb_cmc into relax position, current positions of fingers:\n"
            //               << "  - right_thumb_mcp: " << std::abs(currPosition[1]);
        }
    }
       
    // Do position control of right_thumb_cmc
    double currGoal = thumbGoalPos;

    if (powerGraspRight)
    {
        // if (thumbGoalPos > currPosition[0])
        // {
        //     if (thumbGoalPos - currPosition[0] > MAX_STEP_SIZE)
        //         currGoal = currPosition[0] + MAX_STEP_SIZE;   
        // }
        // else
        // {
        //     if (currPosition[0] - thumbGoalPos > MAX_STEP_SIZE)
        //         currGoal = currPosition[0] - MAX_STEP_SIZE;
        // }

        // The PD control law for right_thumb_cmc
        command[0] = thumbKp * (currGoal - currPosition[0]) - thumbKd * currVelocity[0];
    }

    // Do velocity control of right_thumb_cmc
    // double goalVelcity = 2.0;
    // if (thumbGoalPos < currPosition[0])
    // {
    //     goalVelocity = -2.0;
    // }

    // command[0] = thumbKp * (goalVelocity - currVelocity[0])

    // Publish the right hand command
    if(rhCommandPublisher.trylock())
    {
        for (size_t ii = 0; ii < NUM_RIGHT_HAND_DOFS; ii++)
        {   
            rhCommandPublisher.msg_.effort[ii] =  command[ii];
        }

        rhCommandPublisher.msg_.position[0] = currGoal; // publish the current goal position of the right_thumb_cmc

        rhCommandPublisher.unlockAndPublish();
    }

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
    // CONTROLIT_INFO << "Left gripper power grasp: " << (powerGraspLeft ? "TRUE" : "FALSE");
}

void HandControllerDreamer::includeRightPinkyFingerCallback(const boost::shared_ptr<std_msgs::Bool const> & msgPtr)
{
    includeRightPinkyFinger = msgPtr->data;
}

void HandControllerDreamer::includeRightMiddleFingerCallback(const boost::shared_ptr<std_msgs::Bool const> & msgPtr)
{
    includeRightMiddleFinger = msgPtr->data;
}

void HandControllerDreamer::includeRightPointerFingerCallback(const boost::shared_ptr<std_msgs::Bool const> & msgPtr)
{
    includeRightPointerFinger = msgPtr->data;
}

} // namespace dreamer
} // namespace controlit
