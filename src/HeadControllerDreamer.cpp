#include <controlit/dreamer/HeadControllerDreamer.hpp>
#include <controlit/logging/Logging.hpp>

namespace controlit {
namespace dreamer {

#define NUM_DOFS 7

HeadControllerDreamer::HeadControllerDreamer() :
    jointStatePublisher(nullptr)
{
}

HeadControllerDreamer::~HeadControllerDreamer()
{
}

bool HeadControllerDreamer::init(ros::NodeHandle & nh)
{
    currPosition.setZero(NUM_DOFS);
    currVelocity.setZero(NUM_DOFS);

    // CONTROLIT_INFO << "Subscribing to power grasp topic...";
    // rightHandPowerGraspSubscriber = nh.subscribe("controlit/rightHand/powerGrasp", 1, 
    //     & HeadControllerDreamer::rightHandCallback, this);

    // includeRightPinkyFingerSubscriber = nh.subscribe("controlit/rightHand/includeRightPinkyFinger", 1, 
    //     & HeadControllerDreamer::includeRightPinkyFingerCallback, this);

    // includeRightMiddleFingerSubscriber = nh.subscribe("controlit/rightHand/includeRightMiddleFinger", 1, 
    //     & HeadControllerDreamer::includeRightMiddleFingerCallback, this);

    // includeRightPointerFingerSubscriber = nh.subscribe("controlit/rightHand/includeRightIndexFinger", 1, 
    //     & HeadControllerDreamer::includeRightPointerFingerCallback, this);

    // leftGripperPowerGraspSubscriber = nh.subscribe("controlit/leftGripper/powerGrasp", 1,
    //     & HeadControllerDreamer::leftGripperCallback, this);

    // CONTROLIT_INFO << "Subscribed to topic: " << rightHandPowerGraspSubscriber.getTopic();

    // Create a real-time publisher of the joint states.
    jointStatePublisher.reset(
        new controlit::addons::ros::RealtimePublisherHeader<sensor_msgs::JointState>(nh, "head/joint_states", 1));
    if(jointStatePublisher->trylock())
    {
        jointStatePublisher->msg_.name.push_back("lower_neck_pitch");
        jointStatePublisher->msg_.name.push_back("upper_neck_yaw");
        jointStatePublisher->msg_.name.push_back("upper_neck_roll");
        jointStatePublisher->msg_.name.push_back("upper_neck_pitch");
        jointStatePublisher->msg_.name.push_back("eye_pitch");
        jointStatePublisher->msg_.name.push_back("right_eye_yaw");
        jointStatePublisher->msg_.name.push_back("left_eye_yaw");

        for (size_t ii = 0; ii < NUM_DOFS; ii++)
        {
            jointStatePublisher->msg_.position.push_back(0.0);  // allocate memory for the joint states
            jointStatePublisher->msg_.velocity.push_back(0.0);
            jointStatePublisher->msg_.effort.push_back(0.0);
        }

        jointStatePublisher->unlockAndPublish();
    }

    return true;
}

void HeadControllerDreamer::updateState(Vector position, Vector velocity)
{
    currPosition = position;
    currVelocity = velocity;

    // Publish the current joint state
    if(jointStatePublisher->trylock())
    {
        for (size_t ii = 0; ii < NUM_DOFS; ii++)
        {
            jointStatePublisher->msg_.position[ii] = position[ii];
            jointStatePublisher->msg_.velocity[ii] = velocity[ii];
        }
        
        jointStatePublisher->unlockAndPublish();
    }
}

void HeadControllerDreamer::getCommand(Vector & command)
{
    command.setZero(NUM_DOFS); // for debugging, reset everything to zero 
}

// void HeadControllerDreamer::rightHandCallback(const boost::shared_ptr<std_msgs::Bool const> & msgPtr)
// {
//     powerGraspRight = msgPtr->data;
//     // CONTROLIT_INFO << "Right hand power grasp: " << (powerGraspRight ? "TRUE" : "FALSE");
// }

// void HeadControllerDreamer::leftGripperCallback(const boost::shared_ptr<std_msgs::Bool const> & msgPtr)
// {
//     powerGraspLeft = msgPtr->data;
//     // CONTROLIT_INFO << "Left gripper power grasp: " << (powerGraspLeft ? "TRUE" : "FALSE");
// }

// void HeadControllerDreamer::includeRightPinkyFingerCallback(const boost::shared_ptr<std_msgs::Bool const> & msgPtr)
// {
//     includeRightPinkyFinger = msgPtr->data;
// }

// void HeadControllerDreamer::includeRightMiddleFingerCallback(const boost::shared_ptr<std_msgs::Bool const> & msgPtr)
// {
//     includeRightMiddleFinger = msgPtr->data;
// }

// void HeadControllerDreamer::includeRightPointerFingerCallback(const boost::shared_ptr<std_msgs::Bool const> & msgPtr)
// {
//     includeRightPointerFinger = msgPtr->data;
// }

} // namespace dreamer
} // namespace controlit
