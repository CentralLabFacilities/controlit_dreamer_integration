#include <controlit/dreamer/ServoClockDreamer.hpp>
#include <controlit/logging/RealTimeLogging.hpp>

namespace controlit {
namespace dreamer {

// Uncomment one of the following lines to enable/disable detailed debug statements.
// #define PRINT_INFO_STATEMENT(ss)
#define PRINT_INFO_STATEMENT(ss) CONTROLIT_INFO_RT << ss;

// #define PRINT_INFO_STATEMENT_RT(ss)
#define PRINT_INFO_STATEMENT_RT(ss) CONTROLIT_INFO_RT << ss;

#define NON_REALTIME_PRIORITY 1
#define MAX_START_LATENCY_CYCLES 30

/*!
 * This global method takes as input a pointer to a ServoClockDreamer
 * object and calls rtMethod() on it. It is necessary to be compatible with
 * rt_thread_create().
 *
 * \param[in] scd A pointer to the ServoClockDreamer class.
 * \return the return value of the call to ServoClockDreamer->rtMethod().
 */
void * call_rtMethod(void * scd)
{
    ServoClockDreamer * servoClock = static_cast<ServoClockDreamer*>(scd);
    return servoClock->rtMethod(nullptr);
}

ServoClockDreamer::ServoClockDreamer() :
    ServoClock(), // Call super-class' constructor
    rtThreadState(RT_THREAD_UNDEF)
{
    PRINT_INFO_STATEMENT("ServoClockDreamer Created");
}

ServoClockDreamer::~ServoClockDreamer()
{
}

void ServoClockDreamer::updateLoopImpl()
{
    PRINT_INFO_STATEMENT_RT("Method called!");

    // Compute the period of the real-time servo loop.
    // TODO: Make this a parameter
    rtPeriod_ns = 1000000000L / frequency;
    long long const rtPeriod_us(rtPeriod_ns / 1000);
    
    // Change scheduler of this thread to be RTAI
    rt_allow_nonroot_hrt();
    RT_TASK * normalTask = rt_task_init_schmod(nam2num("TSHM"), NON_REALTIME_PRIORITY, 0, 0, SCHED_FIFO, 0xF);
    if (!normalTask)
        throw std::runtime_error("rt_task_init_schmod failed for non-RT task");
    
    // Spawn the real-time thread
    CONTROLIT_INFO_RT << "Spawning RT thread";
    rtThreadState = RT_THREAD_UNDEF;
    int rtThreadID = rt_thread_create((void*)call_rtMethod,
                                  this,   // parameters
                                  10000); // stack size

    // Wait up to MAX_START_LATENCY_CYCLES for real-time thread to begin running
    for (int ii = 0; ii < MAX_START_LATENCY_CYCLES; ii++) 
    {
        if (rtThreadState == RT_THREAD_RUNNING || rtThreadState == RT_THREAD_ERROR) 
        {
            break;
        }
        usleep(rtPeriod_us);
    }

    CONTROLIT_INFO_RT << "RT thread started!";
    
    if (rtThreadState != RT_THREAD_RUNNING) 
    {
        std::stringstream ss;
        ss << "Invalid real-time thread state: ";

        switch (rtThreadState) 
        {
            case RT_THREAD_UNDEF: ss << "RT_THREAD_UNDEF"; break;
            case RT_THREAD_INIT: ss << "RT_THREAD_INIT"; break;
            case RT_THREAD_RUNNING: ss << "RT_THREAD_RUNNING"; break;
            case RT_THREAD_CLEANUP: ss << "RT_THREAD_CLEANUP"; break;
            case RT_THREAD_ERROR: ss << "RT_THREAD_ERROR"; break;
            case RT_THREAD_DONE: ss << "RT_THREAD_DONE"; break;
            default: ss << "Invalid state " << rtThreadState;
        }

        usleep(15 * rtPeriod_us);
        rt_task_delete(normalTask);
        rt_thread_join(rtThreadID);  // blocks until the real-time thread exits.
        throw std::runtime_error("RT thread failed to start");
    }  
    
    CONTROLIT_INFO_RT << "OK - real-time thread started.";

    rt_thread_join(rtThreadID);  // blocks until the real-time thread exits.
    rt_task_delete(normalTask);
    
    PRINT_INFO_STATEMENT_RT("Method exiting.")
}

void * ServoClockDreamer::rtMethod(void *)
{    
    //////////////////////////////////////////////////
    // Initialize shared memory, RT task, and semaphores.
    
    rtThreadState = RT_THREAD_INIT;
       
    // Switch to use RTAI real-time scheduler
    RT_TASK * task = rt_task_init_schmod(nam2num("TSHMP"), 0, 0, 0, SCHED_FIFO, 0xF);
    rt_allow_nonroot_hrt();
    if (task == nullptr) 
    {
        CONTROLIT_ERROR_RT << "Call to rt_task_init_schmod failed for TSHMP";
        rtThreadState = RT_THREAD_ERROR;
        return nullptr;
    }
    
    // Verify the servo frequency is valid
    if (rtPeriod_ns <= 0) 
    {
        CONTROLIT_ERROR_RT << "Invalid real-time period " << rtPeriod_ns << " ns";
        rtThreadState = RT_THREAD_ERROR;
        rt_task_delete(task);
        return nullptr;
    }
    
    //////////////////////////////////////////////////
    // Start the real time engine...
    
    rtThreadState = RT_THREAD_RUNNING;
    RTIME tickPeriod = nano2count(rtPeriod_ns);
    rt_task_make_periodic(task, rt_get_time() + tickPeriod, tickPeriod); 
    mlockall(MCL_CURRENT | MCL_FUTURE);
    rt_make_hard_real_time();
    
    //////////////////////////////////////////////////
    // The servo loop.

    ros::Time currTime;
    ros::Time prevTime = ros::Time::now();
    ros::Duration duration;
    
    while (continueRunning) 
    {
        rt_task_wait_period();
        long long const start_time(nano2count(rt_get_cpu_time_ns()));
        
        currTime = ros::Time::now();
        duration = currTime - prevTime;
        servoableClass->update(currTime, duration);
        prevTime = currTime;
  
        long long const end_time(nano2count(rt_get_cpu_time_ns()));
        long long const dt(end_time - start_time);
        if (dt > tickPeriod) 
        {
            CONTROLIT_WARN_RT << "Slowing period of RT task from " << count2nano(tickPeriod) << " to " << count2nano(dt);
    
            tickPeriod = dt;
            rtPeriod_ns = count2nano(dt);
            rt_task_make_periodic(task, rt_get_time() + tickPeriod, tickPeriod); 
        }
    }
    
    //////////////////////////////////////////////////
    // Clean up after ourselves.
    
    CONTROLIT_INFO_RT << "Exiting RT thread";
    
    rtThreadState = RT_THREAD_CLEANUP;
    rt_make_soft_real_time();
    rt_task_delete(task);
    return nullptr;
}

} // namespace dreamer
} // namespace controlit
