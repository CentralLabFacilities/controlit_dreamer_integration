#include <controlit/dreamer/ServoClockDreamerTester.hpp>

namespace controlit {
namespace dreamer {

#define TEST_PERIOD 10

ServoClockDreamerTester::ServoClockDreamerTester() :
  initialized(false)
{
}

ServoClockDreamerTester::~ServoClockDreamerTester()
{
}

bool ServoClockDreamerTester::init()
{
    servoClock.init(this);
    initialized = true;
    return true;
}

bool ServoClockDreamerTester::start()
{
    prevTime = ros::Time::now();
    double freq = 1000; // TODO: make this a command line parameter
    servoClock.start(freq);
    return true;
}

bool ServoClockDreamerTester::stop()
{
    servoClock.stop();
    return true;
}

void ServoClockDreamerTester::servoInit()
{
    std::cerr << "Servo init called!" << std::endl;
}

void ServoClockDreamerTester::servoUpdate()
{
    ros::Time currTime = ros::Time::now();
    double elapsedTimeMS = (currTime - prevTime).toSec() * 1000;
    std::cerr << "Method called, elapsed time = " << elapsedTimeMS << " ms, jitter = " << (elapsedTimeMS - 1) * 1000 << " us" << std::endl;
    prevTime = currTime;
}

std::string ServoClockDreamerTester::toString(std::string const& prefix) const
{
    ros::NodeHandle nh;

    std::stringstream ss;
    ss << prefix << "ServoClockDreamerTester details:\n";
    ss << prefix << "  - initialized: " << (initialized ? "true" : "false");

    return ss.str();
}

} // namespace servo_clock_library
} // namespace controlit


// This is the main method that starts everything.
int main(int argc, char **argv)
{
    // Define usage
    std::stringstream ss;
    ss << "Usage: rosrun controlit_dreamer_integration ServoClockDreamerTester [options]\n"
       << "Valid options include:\n"
       << "  -h: display this usage string";

    ros::init(argc, argv, "ServoClockDreamerTester");

    if (argc != 1)
    {
        // Parse the command line arguments
        int option_char;
        while ((option_char = getopt (argc, argv, "h")) != -1)
        {
            switch (option_char)
            {
                case 'h':
                    std::cout << ss.str() << std::endl;
                    return 0;
                    break;
                default:
                    std::cerr << "ERROR: Unknown option " << option_char << ".  " << ss.str() << std::endl;
                    return -1;
            }
        }
    }

    ros::NodeHandle nh;

    std::cout << "ServoClockDreamerTester: Starting test..." << std::endl;

    // Create and start a ServoClockDreamerTester
    controlit::dreamer::ServoClockDreamerTester servoClockDreamerTester;
    servoClockDreamerTester.init();
    servoClockDreamerTester.start();

    // Loop until someone hits ctrl+c
    ros::Rate loop_rate(1);
    int loopCounter = 0;

    std::cout << "ServoClockDreamerTester: Letting test run for " << TEST_PERIOD << " seconds." << std::endl;
    while (ros::ok() && loopCounter < TEST_PERIOD)
    {
        ros::spinOnce();
        loop_rate.sleep();
    }

    std::cout << "ServoClockDreamerTester: Done test, stopping servo clock." << std::endl;

    // Stop ServoClockDreamerTester
    servoClockDreamerTester.stop();
}
