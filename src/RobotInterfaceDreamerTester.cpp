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

#include <controlit/dreamer/RobotInterfaceDreamerTester.hpp>

namespace controlit {
namespace dreamer {

#define TEST_PERIOD 10

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
    jointNames.push_back("m3joint_mt3_j0");
    jointNames.push_back("m3joint_mt3_j1");
    jointNames.push_back("m3joint_slave_mt3_j2");
    jointNames.push_back("left_arm_pan");
    jointNames.push_back("left_shoulder_pitch");
    jointNames.push_back("left_shoulder_roll");
    jointNames.push_back("left_arm_elbow");
    jointNames.push_back("left_arm_elbow_rotator");
    jointNames.push_back("left_wrist_j0");
    jointNames.push_back("left_wrist_j1");
    jointNames.push_back("m3joint_ma10_j0");
    jointNames.push_back("m3joint_ma10_j1");
    jointNames.push_back("m3joint_ma10_j2");
    jointNames.push_back("m3joint_ma10_j3");
    jointNames.push_back("m3joint_ma10_j4");
    jointNames.push_back("m3joint_ma10_j5");
    jointNames.push_back("m3joint_ma10_j6");
    jointNames.push_back("m3joint_ms2_j0");
    jointNames.push_back("m3joint_ms2_j1");
    jointNames.push_back("m3joint_ms2_j2");
    jointNames.push_back("m3joint_ms2_j3");
   
    std::cout << "Initializing the robot state..." << std::endl; 
    robotState.init(jointNames);

    std::cout << "Done initializing the robot interface tester..." << std::endl;

    return true;
}

bool RobotInterfaceDreamerTester::start()
{
    double freq = 1; // TODO: make this a command line parameter
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
void RobotInterfaceDreamerTester::servoUpdate(const ros::Time & time, const ros::Duration & period)
{
    std::cout << "Calling robotInterface.read()..." << std::endl;
    if (!robotInterface.read(ros::Time::now(), robotState))
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

    std::cout << "RobotInterfaceDreamerTester: Starting test..." << std::endl;

    // Create and start a RobotInterfaceDreamerTester
    controlit::dreamer::RobotInterfaceDreamerTester RobotInterfaceDreamerTester;
    RobotInterfaceDreamerTester.init();
    RobotInterfaceDreamerTester.start();

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
