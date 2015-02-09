#include <controlit/dreamer/ServoClockDreamerTester.hpp>

#include <string>

namespace controlit {
namespace dreamer {

#define TEST_PERIOD 60 // Number of seconds to run test
#define DEFAULT_SERVO_FREQUENCY 1000  // In Hz

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

bool ServoClockDreamerTester::start(double freq)
{
    timer.start();
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
    double elapsedTimeMS = timer.getTime() * 1000;
    timer.start();
    std::cerr << "Method called, elapsed time = " << elapsedTimeMS << " ms, jitter = " << (elapsedTimeMS - 1) * 1000 << " us" << std::endl;
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
       << "  -h: display this usage string\n"
       << "  -f [frequency]: the desired servo frequency (default: " << DEFAULT_SERVO_FREQUENCY << "Hz)";

    ros::init(argc, argv, "ServoClockDreamerTester");

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

    ros::NodeHandle nh;

    std::cout << "ServoClockDreamerTester: Starting test..." << std::endl;

    // Create and start a ServoClockDreamerTester
    controlit::dreamer::ServoClockDreamerTester servoClockDreamerTester;
    servoClockDreamerTester.init();
    servoClockDreamerTester.start(freq);

    // Loop at 1Hz until someone hits ctrl+c
    ros::Rate loop_rate(1);
    int loopCounter = 0;

    std::cout << "ServoClockDreamerTester: Letting test run for " << TEST_PERIOD << " seconds." << std::endl;
    while (ros::ok() && loopCounter++ < TEST_PERIOD)
    {
        ros::spinOnce();
        loop_rate.sleep();
    }

    std::cout << "ServoClockDreamerTester: Done test, stopping servo clock." << std::endl;

    // Stop ServoClockDreamerTester
    servoClockDreamerTester.stop();
}
