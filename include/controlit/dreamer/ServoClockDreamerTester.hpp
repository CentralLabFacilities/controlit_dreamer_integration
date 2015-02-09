#ifndef __CONTROLIT_SERVO_CLOCK_DREAMER_TESTER_HPP__
#define __CONTROLIT_SERVO_CLOCK_DREAMER_TESTER_HPP__

#include "ros/ros.h"
#include <controlit/dreamer/ServoClockDreamer.hpp>
#include <controlit/dreamer/TimerRTAI.hpp>
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
     *
     * \param[in] freq The desired servo frequency in Hz.
     */
    bool start(double freq);

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
     * The timer used to measure the servo period.
     */
    TimerRTAI timer;
};

} // namespace dreamer
} // namespace controlit

#endif  // __CONTROLIT_SERVO_CLOCK_DREAMER_TESTER_HPP__
