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
#ifndef __CONTROLIT_ROBOT_INTERFACE_DREAMER_TESTER_HPP__
#define __CONTROLIT_ROBOT_INTERFACE_DREAMER_TESTER_HPP__

#include "ros/ros.h"
#include <controlit/servo_clock_library/ServoClockDreamer.hpp>
#include <controlit/ServoableClass.hpp>
#include <controlit/dreamer/RobotInterfaceDreamer.hpp>
#include <controlit/RTControlModel.hpp>

namespace controlit {
namespace dreamer {

/*!
 * A class that provides the main method for launching a ControlIt!
 * controller.
 */
class RobotInterfaceDreamerTester : controlit::ServoableClass
{
public:
    /*!
     * The default constructor.
     */
    RobotInterfaceDreamerTester();

    /*!
     * The destructor.
     */
    virtual ~RobotInterfaceDreamerTester();

    /*!
     * Initializes this tester.
     *
     * \return Whether the initialization was successful.
     */
    bool init();

    /*!
     * Starts this tester.
     */
    bool start();

    /*!
     * Stops this tester.
     */
    bool stop();

    /*!
     * The callback function for the Servo Clock.
     */
    virtual void update(const ros::Time & time, const ros::Duration & period);

private:

    /*!
     * Whether the instantiation of this class is initialized.
     */
    bool initialized;

    /*!
     * Whether this is the first time the servo clock calls update.
     */
    bool firstRound;

    /*!
     * A ROS node handle.  Used during the initialization of the robot interface.
     */
    ros::NodeHandle nh;

    /*!
     * The model that's used during the initialization of the robot interface.
     */
    controlit::RTControlModel model;

    /*!
     * The name of the controller.  All parameters for the controller
     * should reside on the ROS parameter server under a namespace
     * with the same name as this controller.
     */
    // std::string controllerName;

    /*!
     * For holding the robot state that's read from the robot interface.
     */
    controlit::RobotState robotState;

    /*!
     * The servo clock that is being tested.
     */
    controlit::servo_clock_library::ServoClockDreamer servoClock;

    /*!
     * The robot interface being tested.
     */
    controlit::dreamer::RobotInterfaceDreamer robotInterface;

    ros::Time prevTime;
};

} // namespace dreamer
} // namespace controlit

#endif  // __CONTROLIT_ROBOT_INTERFACE_DREAMER_TESTER_HPP__
