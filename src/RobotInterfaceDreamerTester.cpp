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

#include <controlit/servo_clock_library/RobotInterfaceDreamerTester.hpp>
// #include <controlit/logging/RealTimeLogging.hpp>
// #include <controlit/addons/ros/ROSParameterAccessor.hpp>
// #include <controlit/servo_clock_library/ServoClockDreamer.hpp>

#include <ros/ros.h>
#include <controlit/dreamer/RobotInterfaceDreamer.hpp>

// This is the main method that starts everything.
int main(int argc, char **argv)
{
    // Define usage
    std::stringstream ss;
    ss << "Usage: rosrun controlit_dreamer_integration RobotInterfaceDreamerTester [options]\n"
       << "Valid options include:\n"
       << "  -h: display this usage string";

    ros::init(argc, argv, "RobotInterfaceDreamerTester");

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

    ros::AsyncSpinner spinner(1);
    spinner.start();

    ros::NodeHandle nh;

    std::cout << "RobotInterfaceDreamerTester: Starting test..." << std::endl;

    // Instantiate a RTControlModel and a RobotInterfaceDreamer object
    RTControlModel model;
    controlit::dreamer::RobotInterfaceDreamer robotInterfaceDreamer;
    robotInterfaceDreamer.init(nh, &model)

    // Instantiate a RobotState object
    controlit::RobotState robotState;

    // Loop until someone hits ctrl+c
    ros::Rate loop_rate(1);
    int loopCounter = 0;

    std::cout << "RobotInterfaceDreamerTester: Running test..." << std::endl;
    while (ros::ok())
    {
        bool block = false;
        robotIntefaceDreamer.read(ros::Time::now(), robotState, block);
        loop_rate.sleep();
    }

    std::cout << "RobotInterfaceDreamerTester: Done test..." << std::endl;
}