#include <controlit/dreamer/TimerRTAI.hpp>

#include <rtai_shm.h> // for rt_get_cpu_time_ns
//#include <rtai_sched.h>

namespace controlit {
namespace dreamer {

TimerRTAI::TimerRTAI()
{
}

void TimerRTAI::start()
{
    startTime = rt_get_cpu_time_ns();;
}

double TimerRTAI::getTime()
{
    return (rt_get_cpu_time_ns() - startTime) / 1e9;
}

} // namespace dreamer
} // namespace controlit
