#include <controlit/servo_clock_library/ServoClockDreamer.hpp>
#include <controlit/logging/RealTimeLogging.hpp>

namespace controlit {
namespace servo_clock_library {

// Uncomment one of the following lines to enable/disable detailed debug statements.
// #define PRINT_INFO_STATEMENT(ss)
#define PRINT_INFO_STATEMENT(ss) CONTROLIT_INFO_RT << ss;

// #define PRINT_INFO_STATEMENT_RT(ss)
#define PRINT_INFO_STATEMENT_RT(ss) CONTROLIT_INFO_RT << ss;

#define NON_REALTIME_PRIORITY 1
#define MAX_START_LATENCY_CYCLES 30

/*!
 * This is a global method that takes as input a pointer to a ServoClockDreamer
 * object and calls rtMethod() on it.
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
    // normalTask(nullptr)
{
    PRINT_INFO_STATEMENT("ServoClockDreamer Created");
}

ServoClockDreamer::~ServoClockDreamer()
{
}

void ServoClockDreamer::updateLoopImpl()
{
    PRINT_INFO_STATEMENT_RT("Method called!");

    // Compute the period of the real-time servo loop 
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
                                  this, // parameters
                                  10000); // XXXX 10000 is stack size I think

    // Wait up to MAX_START_LATENCY_CYCLES for real-time thread to begin running
    for (int ii = 0; ii < MAX_START_LATENCY_CYCLES; ii++) 
    {
        // fprintf(stderr, ".");
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

        // shutdown_request = 1;
        usleep(15 * rtPeriod_us);
        rt_task_delete(normalTask);
        rt_thread_join(rtThreadID);  // blocks until the real-time thread exits.
        // normalTask = nullptr;
        // rtThreadID = 0;
        throw std::runtime_error("RT thread failed to start");
    }  
    
    CONTROLIT_INFO_RT << "OK starting real-time thread.";

    PRINT_INFO_STATEMENT_RT("Method exiting.")
}

void * ServoClockDreamer::rtMethod(void *)
{
    // M3Sds * sys;
    // RT_TASK * task = nullptr;
    // SEM * status_sem;
    // SEM * command_sem;
    // RTUtil * rtutil((RTUtil*) arg);
    //M3TorqueShmSdsStatus shm_status;
    //M3TorqueShmSdsCommand shm_cmd;
    // M3UTATorqueShmSdsStatus shm_status;
    // M3UTATorqueShmSdsCommand shm_cmd;
    // jspace::State state(7, 7, 6);
    // jspace::Vector command(7);
    // RTIME tickPeriod;
    // int cb_status;
    
    //////////////////////////////////////////////////
    // Initialize shared memory, RT task, and semaphores.
    
    rtThreadState = RT_THREAD_INIT;
    
    // sys = (M3Sds*) rt_shm_alloc(nam2num(TORQUE_SHM), sizeof(M3Sds), USE_VMALLOC);
    // if (sys) {
    //   fprintf(stderr, "found shared memory\n");
    // }
    // else {
    //   fprintf(stderr, "rt_shm_alloc failed for %s\n", TORQUE_SHM);
    //   rtThreadState = RT_THREAD_ERROR;
    //   goto cleanup_sys;
    // }
    
    // Switch to use RTAI real-time scheduler
    RT_TASK * task = rt_task_init_schmod(nam2num("TSHMP"), 0, 0, 0, SCHED_FIFO, 0xF);
    rt_allow_nonroot_hrt();
    if (task == nullptr) 
    {
        CONTROLIT_ERROR_RT << "Call to rt_task_init_schmod failed for TSHMP";
        rtThreadState = RT_THREAD_ERROR;
        // rt_shm_free(nam2num(TORQUE_SHM));
        return nullptr;
    }
    
    // status_sem = (SEM*) rt_get_adr(nam2num(TORQUE_STATUS_SEM));
    // if ( ! status_sem) {
    //   fprintf(stderr, "semaphore %s not found\n", TORQUE_STATUS_SEM);
    //   rtThreadState = RT_THREAD_ERROR;
    //   goto cleanup_status_sem;
    // }
    
    // command_sem = (SEM*) rt_get_adr(nam2num(TORQUE_CMD_SEM));
    // if ( ! command_sem) {
    //   fprintf(stderr, "semaphore %s not found\n", TORQUE_CMD_SEM);
    //   rtThreadState = RT_THREAD_ERROR;
    //   goto cleanup_command_sem;
    // }
    
    //////////////////////////////////////////////////
    // Give the user a chance to do stuff before we enter periodic
    // hard real time.
    
    // rt_sem_wait(status_sem);
    // memcpy(&shm_status, sys->status, sizeof(shm_status));
    // rt_sem_signal(status_sem);
    // for (size_t ii(0); ii < 7; ++ii) { // XXXX to do: hardcoded NDOF
    //   state.position_[ii] = M_PI * shm_status.right_arm.theta[ii] / 180.0;
    //   state.velocity_[ii] = M_PI * shm_status.right_arm.thetadot[ii] / 180.0;
    //   //state.force_[ii] = 1.0e-3 * shm_status.right_arm.torque[ii];
    // }
        
    ///Force-Torque Sensor
    // for (size_t jj(0); jj < 6; ++jj) {
    //   state.force_[jj] = 1.0e-3 * shm_status.right_arm.wrench[jj];
    // }   

    // cb_status = rtutil->init(state);
    // if (0 != cb_status) {
    //   fprintf(stderr, "init callback returned %d\n", cb_status);
    //   rtThreadState = RT_THREAD_ERROR;
    //   goto cleanup_init_callback;
    // }
    
    // Verify the servo frequency is valid
    if (rtPeriod_ns <= 0) 
    {
        CONTROLIT_ERROR_RT << "Invalid real-time period " << rtPeriod_ns << " ns";
        rtThreadState = RT_THREAD_ERROR;
        rt_task_delete(task);
        // rt_shm_free(nam2num(TORQUE_SHM));
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
    // The loop.

    ros::Time currTime;
    ros::Time prevTime = ros::Time::now();
    ros::Duration duration;
    
    while (continueRunning) 
    {
        rt_task_wait_period();
        long long const start_time(nano2count(rt_get_cpu_time_ns()));
        
        // rt_sem_wait(status_sem);
        // memcpy(&shm_status, sys->status, sizeof(shm_status));
        // rt_sem_signal(status_sem);
        // for (size_t ii(0); ii < 7; ++ii) { // XXXX to do: hardcoded NDOF
        //   state.position_[ii] = M_PI * shm_status.right_arm.theta[ii] / 180.0;
        //   state.velocity_[ii] = M_PI * shm_status.right_arm.thetadot[ii] / 180.0;
        // }
  
        ///Force-Torque Sensor
        // for (size_t jj(0); jj < 6; ++jj) {
        //   state.force_[jj] = 1.0e-3 * shm_status.right_arm.wrench[jj];
        // }
        ///
  
        // cb_status = rtutil->update(state, command);
        // if (0 != cb_status) {
        //   fprintf(stderr, "update callback returned %d\n", cb_status);
        //   rtThreadState = RT_THREAD_ERROR;
        //   shutdown_request = 1;
        //   continue;
        // }
        
        // for (size_t ii(0); ii < 7; ++ii) { // XXXX to do: hardcoded NDOF
        //   shm_cmd.right_arm.tq_desired[ii] = 1.0e3 * command[ii];
        // }
        // shm_cmd.timestamp = shm_status.timestamp;
        // rt_sem_wait(command_sem);
        // memcpy(sys->cmd, &shm_cmd, sizeof(shm_cmd));              
        // rt_sem_signal(command_sem);
        
        currTime = ros::Time::now();
        duration = currTime - prevTime;
        servoableClass->update(currTime, duration);
        prevTime = currTime;
  
        long long const end_time(nano2count(rt_get_cpu_time_ns()));
        long long const dt(end_time - start_time);
        if (dt > tickPeriod) 
        {
            // cb_status = rtutil->slowdown(step_cnt,
            //                              count2nano(tickPeriod),
            //                              count2nano(dt));
            // if (0 != cb_status) {
            //   fprintf(stderr, "slowdown callback returned %d\n"
            //           "  iteration: %lld\n"
            //           "  desired period: %lld ns\n"
            //           "  actual period: %lld ns\n",
            //           cb_status, step_cnt, count2nano(tickPeriod), count2nano(dt));
            //   rtThreadState = RT_THREAD_ERROR;
            //   shutdown_request = 1;
            //   continue;
            // }
            CONTROLIT_ERROR_RT << "Slowing period of RT task from " << count2nano(tickPeriod) << " to " << count2nano(dt);
    
            tickPeriod = dt;
            rtPeriod_ns = count2nano(dt);
            rt_task_make_periodic(task, rt_get_time() + tickPeriod, tickPeriod); 
        }
    } // end "the big for loop"
    
    //////////////////////////////////////////////////
    // Clean up after ourselves
    
    CONTROLIT_INFO_RT << "Exiting RT thread";
    
    rtThreadState = RT_THREAD_CLEANUP;
    rt_make_soft_real_time();
    
    // cb_status = rtutil->cleanup();
    // if (cb_status != 0) 
    // {
    //     CONTROLIT_ERROR_RT << "Cleanup returned " << cb_status;
    //     rtThreadState = RT_THREAD_ERROR;
    // }
    // else 
    // {
    //     rtThreadState = RT_THREAD_DONE;
    // }
    
  // cleanup_period_check:
  // cleanup_init_callback:
  // cleanup_command_sem:
  // cleanup_status_sem:
    rt_task_delete(task);
  // cleanup_task:
    // rt_shm_free(nam2num(TORQUE_SHM));
  // cleanup_sys:

    return nullptr;
}

} // namespace servo_clock_library
} // namespace controlit
