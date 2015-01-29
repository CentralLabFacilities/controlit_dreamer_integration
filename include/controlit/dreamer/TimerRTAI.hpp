#ifndef __CONTROLIT_DREAMER_INTEGRATION_TIMER_RTAI_HPP__
#define __CONTROLIT_DREAMER_INTEGRATION_TIMER_RTAI_HPP__

#include <ros/ros.h>
#include <controlit/Timer.hpp>

namespace controlit {
namespace dreamer {

class TimerRTAI : public Timer
{
public:
    /*!
     * The default constructor.
     */
    explicit TimerRTAI();
  
    /*!
     * Starts the timer.
     */
    virtual void start();

    /*!
     * Gets the timer's current value in milliseconds.
     *
     * \return The number of milliseconds that has elapsed since
     * the call to start().
     */
    virtual double getTime();

private:
    RTIME startTime;
};

} // namespace dreamer
} // namespace controlit

#endif // __CONTROLIT_DREAMER_INTEGRATION_TIMER_RTAI_HPP__

