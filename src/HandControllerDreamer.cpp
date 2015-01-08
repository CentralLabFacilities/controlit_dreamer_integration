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

#define POWER_GRASP_ENABLED_KP 3.5
#define POWER_GRASP_DISABLED_KP 3.0

HandControllerDreamer::HandControllerDreamer() :
    powerGraspRight(false),
    powerGraspLeft(false),
    closingRightFingers(false),
    includeRightPointerFinger(true),
    includeRightMiddleFinger(true),
    includeRightPinkyFinger(true),
    thumbKp(POWER_GRASP_DISABLED_KP),
    thumbKd(0),
    thumbGoalPos(0)
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

    includeRightPointerFingerSubscriber = nh.subscribe("controlit/rightHand/includePointerPinkyFinger", 1, 
        & HandControllerDreamer::includeRightPointerFingerCallback, this);

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
            command[2] = (includeRightPointerFinger ? 0.3 : -0.1); // right_pointer_finger
            command[3] = (includeRightMiddleFinger ? 0.3 : -0.1);  // right_middle_finger
            command[4] = (includeRightPinkyFinger ? 0.3 : -0.1);   // right_pinky_finger
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
            thumbKp = POWER_GRASP_DISABLED_KP;
            thumbGoalPos = 1.57;  // right_thumb_cmc 180 degrees from palm
        }
        else
        {
            // CONTROLIT_INFO << "Not putting right_thumb_cmc into relax position, current positions of fingers:\n"
            //               << "  - right_thumb_mcp: " << std::abs(currPosition[1]);
        }
    }
       
    // Do position control of right_thumb_cmc
    double currGoal = thumbGoalPos;

    if (thumbGoalPos > currPosition[0])
    {
        if (thumbGoalPos - currPosition[0] > MAX_STEP_SIZE)
            currGoal = currPosition[0] + MAX_STEP_SIZE;   
    }
    else
    {
        if (currPosition[0] - thumbGoalPos > MAX_STEP_SIZE)
            currGoal = currPosition[0] - MAX_STEP_SIZE;
    }

    // The PD control law for right_thumb_cmc
    command[0] = thumbKp * (currGoal - currPosition[0]) - thumbKd * currVelocity[0];

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
