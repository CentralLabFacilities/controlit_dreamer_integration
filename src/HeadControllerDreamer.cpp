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

    commandPos.setZero(NUM_DOFS);
    commandVel.setZero(NUM_DOFS);

    // Create a real-time publisher of the joint states.
    jointStatePublisher.reset(
        new controlit::addons::ros::RealtimePublisherHeader<sensor_msgs::JointState>(nh, "controlit/head/joint_states", 1));
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

    // Create a subscriber for the lower neck pitch command
    lowerNeckPitchSubscriber = nh.subscribe("controlit/head/lower_neck_pitch/position_cmd", 1, 
        & HeadControllerDreamer::lowerNeckPitchCallback, this);

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
    // CONTROLIT_INFO << "Method Called";

    for (size_t ii = 0; ii < NUM_DOFS; ii++)
    {
        // CONTROLIT_INFO << "Setting command [" << ii << "] to be: " << commandPos[ii];
        command[ii] = commandPos[ii];   
    }
    // command.setZero(NUM_DOFS); // for debugging, reset everything to zero 
}

void HeadControllerDreamer::lowerNeckPitchCallback(const boost::shared_ptr<std_msgs::Float64 const> & msgPtr)
{
    commandPos[0] = msgPtr->data;
}

} // namespace dreamer
} // namespace controlit
