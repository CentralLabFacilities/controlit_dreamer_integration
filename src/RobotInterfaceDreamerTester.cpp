#include <controlit/dreamer/RobotInterfaceDreamerTester.hpp>

namespace controlit {
namespace dreamer {

#define TEST_PERIOD 60 // Number of seconds to run test
#define DEFAULT_SERVO_FREQUENCY 1000  // In Hz

RobotInterfaceDreamerTester::RobotInterfaceDreamerTester()
{
}

RobotInterfaceDreamerTester::~RobotInterfaceDreamerTester()
{
}

bool RobotInterfaceDreamerTester::init()
{
    std::cout << "RobotInterfaceDreamerTester Init Method called." << std::endl;
    // Initialize the robot interface.
    if (!robotInterface.init(nh, &model))
    {
        std::cerr << "Problems initializing the robot interface." << std::endl;
        ros::shutdown();
        return false;
    }

    // Initialize the servo clock.
    if (!servoClock.init(this))
    {
        std::cerr << "Problems initializing servo clock." << std::endl;
        ros::shutdown();
        return false;
    }

    // Initialize the RobotState
    std::vector<std::string> jointNames;
    jointNames.push_back("torso_yaw");
    jointNames.push_back("torso_lower_pitch");
    jointNames.push_back("torso_upper_pitch");
    jointNames.push_back("left_shoulder_extensor");
    jointNames.push_back("left_shoulder_abductor");
    jointNames.push_back("left_shoulder_rotator");
    jointNames.push_back("left_elbow");
    jointNames.push_back("left_wrist_rotator");
    jointNames.push_back("left_wrist_pitch");
    jointNames.push_back("left_wrist_yaw");
    jointNames.push_back("right_shoulder_extensor");
    jointNames.push_back("right_shoulder_abductor");
    jointNames.push_back("right_shoulder_rotator");
    jointNames.push_back("right_elbow");
    jointNames.push_back("right_wrist_rotator");
    jointNames.push_back("right_wrist_pitch");
    jointNames.push_back("right_wrist_yaw");
    jointNames.push_back("lower_neck_pitch");
    jointNames.push_back("upper_neck_yaw");
    jointNames.push_back("upper_neck_roll");
    jointNames.push_back("upper_neck_pitch");
   
    std::cout << "Initializing the robot state..." << std::endl; 
    robotState.init(jointNames);

    publisher.init(nh, "robotInterfaceDreamerTester/frequency", 1);
    if(publisher.trylock())
    {
        publisher.msg_.data = 0;
        publisher.unlockAndPublish();
    }
    else
    {
        std::cerr << "ServoClockDraemerTester::init: ERROR: Unable to initialize publisher!" << std::endl;
        return false;
    }

    std::cout << "Done initializing the robot interface tester..." << std::endl;

    return true;
}

bool RobotInterfaceDreamerTester::start(double freq)
{
    timer.start();
    servoClock.start(freq);
    return true;
}

bool RobotInterfaceDreamerTester::stop()
{
    servoClock.stop();
    return true;
}

void RobotInterfaceDreamerTester::servoInit()
{
    std::cerr << "Servo init called!" << std::endl;
}


// This is periodically called by the servo clock.
void RobotInterfaceDreamerTester::servoUpdate()
{
    double elapsedTime = timer.getTime();
    timer.start();
    
    if (publisher.trylock())
    {
        publisher.msg_.data = 1.0 / elapsedTime;
        publisher.unlockAndPublish();
    }

    // std::cout << "Calling robotInterface.read()..." << std::endl;
    if (!robotInterface.read(robotState))
    {
        std::cerr << "Problems reading from robot state." << std::endl;
    }
}

} // namespace dreamer
} // namespace controlit


// This is the main method that starts everything.
int main(int argc, char **argv)
{
    // Define usage
    std::stringstream ss;
    ss << "Usage: rosrun controlit_dreamer_integration RobotInterfaceDreamerTester [options]\n"
       << "Valid options include:\n"
       << "  -h: display this usage string";

    ros::init(argc, argv, "RobotInterfaceDreamerTester");

    double freq = DEFAULT_SERVO_FREQUENCY;

    if (argc != 1)
    {
        // Parse the command line arguments
        int option_char;
        while ((option_char = getopt (argc, argv, "hf:")) != -1)
        {
            switch (option_char)
            {
                case 'h':
                    std::cout << ss.str() << std::endl;
                    return 0;
                    break;
                case 'f':
                    freq = std::stod(optarg);
                    break;
                default:
                    std::cerr << "ERROR: Unknown option " << option_char << ".  " << ss.str() << std::endl;
                    return -1;
            }
        }
    }

    std::cout << "RobotInterfaceDreamerTester: Starting test, servo frequency = " << freq << "..." << std::endl;

    // Create and start a RobotInterfaceDreamerTester
    controlit::dreamer::RobotInterfaceDreamerTester tester;
    if (!tester.init()) return -1;
    if (!tester.start(freq)) return -1;

    // Loop until someone hits ctrl+c
    ros::Rate loop_rate(1);
    int loopCounter = 0;

    std::cout << "RobotInterfaceDreamerTester: Letting test run for " << TEST_PERIOD << " seconds." << std::endl;
    while (ros::ok() && loopCounter < TEST_PERIOD)
    {
        ros::spinOnce();
        loop_rate.sleep();
    }

    std::cout << "RobotInterfaceDreamerTester: Done test, stopping servo clock." << std::endl;

    // Stop RobotInterfaceDreamerTester
    RobotInterfaceDreamerTester.stop();
}
