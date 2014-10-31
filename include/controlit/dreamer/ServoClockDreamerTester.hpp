/*
 * Shared copyright notice and LGPLv3 license statement.
 *
 * Copyright (C) 2014 University of Texas at Austin. All rights reserved.
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
#include <controlit/dreamer/ServoClockDreamer.hpp>
#include <controlit/ServoableClass.hpp>

namespace controlit {
namespace dreamer {

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

    /*!
     * Called once the first time the servo clock is started.
     */
    void servoInit();

    /*!
     * The callback function for the Servo Clock.
     */
    void servoUpdate();

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
     * The servo clock that is being tested.
     */
    ServoClockDreamer servoClock;

    /*!
     * The previous time update() was called by the servo clock.
     */
    ros::Time prevTime;
};

} // namespace dreamer
} // namespace controlit

#endif  // __CONTROLIT_SERVO_CLOCK_DREAMER_TESTER_HPP__
