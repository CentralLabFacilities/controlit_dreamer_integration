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
#ifndef __CONTROLIT_SERVO_CLOCK_DREAMER_TESTER_HPP__
#define __CONTROLIT_SERVO_CLOCK_DREAMER_TESTER_HPP__

#include "ros/ros.h"
#include <controlit/servo_clock_library/ServoClockDreamer.hpp>
#include <controlit/ServoableClass.hpp>

namespace controlit {
namespace servo_clock_library {

/*!
 * A class that provides the main method for launching a ControlIt!
 * controller.
 */
class ServoClockDreamerTester : controlit::ServoableClass
{
public:
    /*!
     * The default constructor.
     */
    ServoClockDreamerTester();

    /*!
     * The destructor.
     */
    virtual ~ServoClockDreamerTester();

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

    virtual void update(const ros::Time & time, const ros::Duration & period);

    /*!
     * Returns a string representation of this class.
     */
    std::string toString(std::string const & prefix = "") const;

private:

    /*!
     * Whether the instantiation of this class is initialized.
     */
    bool initialized;

    /*!
     * The name of the controller.  All parameters for the controller
     * should reside on the ROS parameter server under a namespace
     * with the same name as this controller.
     */
    // std::string controllerName;

    /*!
     * The servo clock that is being tested.
     */
    std::unique_ptr<controlit::servo_clock_library::ServoClockDreamer> servoClock;

    ros::Time prevTime;
};

} // namespace servo_clock_library
} // namespace controlit

#endif  // __CONTROLIT_SERVO_CLOCK_DREAMER_TESTER_HPP__
