/*
 * Shared copyright notice and LGPLv3 license statement.
 *
 * Copyright (C) 2013 University of Texas at Austin. All rights reserved.
 *
 * Authors: Chien-Liang Fok (UT Austin)
 *
 * Website: http://www.me.utexas.edu/~hcrl/
 *
 * This program is free software: you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public License
 * as published by the Free Software Foundation, either version 3 of
 * the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this program.  If not, see
 * <http://www.gnu.org/licenses/>
 */

#include <controlit/servo_clock_library/ServoClockDreamerTester.hpp>
// #include <controlit/addons/ros/ROSParameterAccessor.hpp>
// #include <controlit/servo_clock_library/ServoClockDreamer.hpp>

namespace controlit {
namespace servo_clock_library {

#define TEST_PERIOD 10

ServoClockDreamerTester::ServoClockDreamerTester() :
  initialized(false),
  servoClock(nullptr)
{
}

ServoClockDreamerTester::~ServoClockDreamerTester()
{
}

bool ServoClockDreamerTester::init()
{
    servoClock.reset(new controlit::servo_clock_library::ServoClockDreamer());
    servoClock->init(this);
    initialized = true;
    return true;
}

bool ServoClockDreamerTester::start()
{
    servoClock->start();
    return true;
}

bool ServoClockDreamerTester::stop()
{
    servoClock->stop();
    return true;
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
    controlit::exec::ServoClockDreamerTester ServoClockDreamerTester;
    ServoClockDreamerTester.init();
    ServoClockDreamerTester.start();

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
    ServoClockDreamerTester.stop();
}