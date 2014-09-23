#include <controlit/servo_clock_library/ServoClockDreamer.hpp>

#include <controlit/logging/RealTimeLogging.hpp>

namespace controlit {
namespace servo_clock_library {

// Uncomment one of the following lines to enable/disable detailed debug statements.
// #define PRINT_INFO_STATEMENT(ss)
#define PRINT_INFO_STATEMENT(ss) CONTROLIT_INFO_RT << ss;

// #define PRINT_INFO_STATEMENT_RT(ss)
#define PRINT_INFO_STATEMENT_RT(ss) CONTROLIT_INFO_RT << ss;
    
ServoClockDreamer::ServoClockDreamer() :
    ServoClock() // Call super-class' constructor
{
	PRINT_INFO_STATEMENT("ServoClockDreamer Created");
}

ServoClockDreamer::~ServoClockDreamer()
{
}

void ServoClockDreamer::updateLoopImpl()
{
    PRINT_INFO_STATEMENT_RT("Method called!");

    // TODO: Add logic for RTAI real-time thread instantiation.
    // The loop should periodically call the following method:
    //
    // servoableClass->update(currTime, duration);

    PRINT_INFO_STATEMENT_RT("Method exiting.")
}

} // namespace servo_clock_library
} // namespace controlit
