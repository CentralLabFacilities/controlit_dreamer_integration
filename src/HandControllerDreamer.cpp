#include <controlit/dreamer/HandControllerDreamer.hpp>
#include <controlit/logging/Logging.hpp>

namespace controlit {
namespace dreamer {

#define NUM_RIGHT_HAND_DOFS 5
#define NUM_COMMAND_DOFS 6                   // Command includes both left and right hands
#define NUM_TORQUE_CONTROLLED_JOINTS 6

#define POWER_GRASP_MODE 0
#define POSITION_MODE 1

#define RIGHT_THUMB_CMC_INDEX 0
#define LEFT_GRIPPER_JOINT_INDEX 5

#define POWER_GRASP_ENABLED_KP 3
#define POWER_GRASP_ENABLED_KD 0

#define THUMB_SPEED 1  // radians per second

#define TORQUE_COMMAND_J0_CONTRACT 0 // temporary value
#define TORQUE_COMMAND_J1_CONTRACT 0.180
#define TORQUE_COMMAND_J2_CONTRACT 0.115
#define TORQUE_COMMAND_J3_CONTRACT 0.2
#define TORQUE_COMMAND_J4_CONTRACT 0.05

#define TORQUE_COMMAND_J0_EXTEND 0
#define TORQUE_COMMAND_J1_EXTEND 0
#define TORQUE_COMMAND_J2_EXTEND 0
#define TORQUE_COMMAND_J3_EXTEND 0
#define TORQUE_COMMAND_J4_EXTEND -0.05

HandControllerDreamer::HandControllerDreamer() :
    rightHandControlMode(POWER_GRASP_MODE),
    powerGraspRight(false),
    powerGraspLeft(false),
    closingThumbFinger(false),
    closingRightFingers(false),
    includeRightPointerFinger(true),
    includeRightMiddleFinger(true),
    includeRightPinkyFinger(true),
    thumbKp(POWER_GRASP_ENABLED_KP),
    thumbKd(POWER_GRASP_ENABLED_KD),
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

    rightHandModeSubscriber = nh.subscribe("controlit/rightHand/mode", 1,
        & HandControllerDreamer::rightHandModeCallback, this);

    rightThumbCMCPosSubscriber = nh.subscribe("controlit/rightHand/thumb/position", 1,
        & HandControllerDreamer::rightThumbCMCPosCallback, this);

    rightThumbCMCKpSubscriber = nh.subscribe("controlit/rightHand/thumb/kp", 1,
        & HandControllerDreamer::rightThumbCMCKpCallback, this);

    rightThumbCMCKdSubscriber = nh.subscribe("controlit/rightHand/thumb/kd", 1,
        & HandControllerDreamer::rightThumbCMCKdCallback, this);

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
            rhStatePublisher.msg_.position[ii] = currPosition[ii];
            rhStatePublisher.msg_.velocity[ii] = currVelocity[ii];
        }

        rhStatePublisher.unlockAndPublish();
    }
}

void HandControllerDreamer::getCommand(Vector & command)
{
    command.setZero(6); // for debugging, reset everything to zero 

    if (rightHandControlMode == POWER_GRASP_MODE)
    {
        if (powerGraspRight)
        {
            // thumbKp = POWER_GRASP_ENABLED_KP;

            // thumbGoalPos = 0; // right_thumb_cmc 90 degrees from palm

            // if (!closingRightFingers && !closingThumbFinger)
            // {
            //     closingThumbFinger = true;

            //     timeBeginCloseThumb = ros::Time::now();
            //     thumbInitPos = currPosition[0];
            // }
            
            // wait until right_thumb_cmc is at the zero position before curling the fingers
            // if (closingRightFingers || std::abs(currPosition[0]) < 0.2)  // 0.2 radians is 11.46 degrees
            // {
                closingRightFingers = true;
                // thumbKp = 1; //POWER_GRASP_DISABLED_KP;

                command[1] = TORQUE_COMMAND_J1_CONTRACT; // close right_thumb_mcp
                command[2] = (includeRightPointerFinger ? TORQUE_COMMAND_J2_CONTRACT : TORQUE_COMMAND_J2_EXTEND);  // right_pointer_finger (uses torque commands with opposite signs)
                command[3] = (includeRightMiddleFinger  ? TORQUE_COMMAND_J3_CONTRACT : TORQUE_COMMAND_J3_EXTEND);  // right_middle_finger
                command[4] = (includeRightPinkyFinger   ? TORQUE_COMMAND_J4_CONTRACT : TORQUE_COMMAND_J4_EXTEND);  // right_pinky_finger
            // }
            // else
            // {
            //     // CONTROLIT_INFO << "Not curling right_thumb_mcp, current position of right_thumb_cmc is " << std::abs(currPosition[0]);

            //     // double elapsedTime = (ros::Time::now() - timeBeginCloseThumb).toSec();
            //     // thumbGoalPos = thumbInitPos - elapsedTime * THUMB_SPEED;
            //     // if (thumbGoalPos < 0)
            //     //     thumbGoalPos = 0;

            //     thumbGoalPos = -0.69; // -40 degrees
            // }

            // The PD control law for right_thumb_cmc
            command[RIGHT_THUMB_CMC_INDEX] = thumbKp * (thumbGoalPos - currPosition[RIGHT_THUMB_CMC_INDEX]) - thumbKd * currVelocity[RIGHT_THUMB_CMC_INDEX];
            // command[0] = -0.15; // Force right_thumb_cmc to go to -40 degree position.
            // command[0] = 0; // send the right_thumb_cmc a torque command of zero to prevent it from overheating
        }
        else
        {
            closingRightFingers = false;
            closingThumbFinger = false;

            // Set the torque commands for "relaxing" e.g. "extending" the fingers"
            command[1] = TORQUE_COMMAND_J1_EXTEND;
            command[2] = TORQUE_COMMAND_J2_EXTEND;
            command[3] = TORQUE_COMMAND_J3_EXTEND;
            command[4] = TORQUE_COMMAND_J4_EXTEND;
            
            // wait until all fingers are relaxed before moving the right_thumb_cmc
            if (std::abs(currPosition[1]) < 0.02)
            {
                command[0] = TORQUE_COMMAND_J0_EXTEND;
                // thumbGoalPos = 1.57;  // right_thumb_cmc at 90 degree position
                // if (currPosition[0] < 1.57)
                // {
                //     command[0] = 0.1; // Issue 0.1 Nm of torque to force the joint to go to position 1.57 radians
                //     timeAtRelaxedPos = ros::Time::now();
                // }
                // else
                // {
                //     double elapsedTime = (ros::Time::now() - timeAtRelaxedPos).toSec();
                //     if (elapsedTime > 3.0)
                //         command[0] = 0.05;
                //     else
                //         command[0] = 0.1;   // Decrease torque after 3 seconds to prevent right_thumb-cmc motor from over heating
                    
                // }
            }
            else
            {
                // CONTROLIT_INFO << "Not putting right_thumb_cmc into relax position, current positions of fingers:\n"
                //               << "  - right_thumb_mcp: " << std::abs(currPosition[1]);
            }
        }
    }
    else // right hand is in position control mode
    {
        command[RIGHT_THUMB_CMC_INDEX] = thumbKp * (thumbGoalPos - currPosition[RIGHT_THUMB_CMC_INDEX]) - thumbKd * currVelocity[RIGHT_THUMB_CMC_INDEX];
    }

    // Publish the right hand command
    if(rhCommandPublisher.trylock())
    {
        for (size_t ii = 0; ii < NUM_RIGHT_HAND_DOFS; ii++)
        {   
            rhCommandPublisher.msg_.effort[ii] =  command[ii];
        }

        rhCommandPublisher.msg_.position[RIGHT_THUMB_CMC_INDEX] = thumbGoalPos; // publish the current goal position of the right_thumb_cmc

        rhCommandPublisher.unlockAndPublish();
    }

    // Issue command to left gripper
    command[5] = powerGraspLeft ? 2 : -0.5;
}


void HandControllerDreamer::rightHandModeCallback(const boost::shared_ptr<std_msgs::Int32 const> & msgPtr)
{
    rightHandControlMode = msgPtr->data;
}

void HandControllerDreamer::rightThumbCMCPosCallback(const boost::shared_ptr<std_msgs::Float64 const> & msgPtr)
{
    thumbGoalPos = msgPtr->data;
}

void HandControllerDreamer::rightThumbCMCKpCallback(const boost::shared_ptr<std_msgs::Float64 const> & msgPtr)
{
    thumbKp = msgPtr->data;
}

void HandControllerDreamer::rightThumbCMCKdCallback(const boost::shared_ptr<std_msgs::Float64 const> & msgPtr)
{
    thumbKd = msgPtr->data;
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
