#ifndef __CONTROLIT_DREAMER_INTEGRATION_SERVO_CLOCK_CHRONO_HPP__
#define __CONTROLIT_DREAMER_INTEGRATION_SERVO_CLOCK_CHRONO_HPP__

#include <controlit/ServoClock.hpp>
#include <thread>  // for std::mutex

namespace controlit {
namespace servo_clock_library {

/*!
 * The coordinator for robots that are controlled via ROS topics.
 *
 * The robot_hardware_interface::ImpedanceCommandInterface type is
 * used as the template type for the Coordinator parent class but is
 * not actually used by this class.
 */
class ServoClockDreamer : public controlit::ServoClock
{
public:
    /*!
     * The constructor.
     */
    ServoClockDreamer();

    /*!
     * The destructor.
     */
    virtual ~ServoClockDreamer();

protected:

    /*!
     * The implementation of the update loop.
     */
    virtual void updateLoopImpl();


};

} // namespace servo_clock_library
} // namespace controlit

#endif // __CONTROLIT_DREAMER_INTEGRATION_SERVO_CLOCK_CHRONO_HPP__
