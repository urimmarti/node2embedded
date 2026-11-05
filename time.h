#pragma once

#include <stddef.h>
#include <stdint.h>
#include <stdbool.h>

// Return the time since the device was started.
uint64_t time_now(void);

// Convert standard wall time units to ticks
uint64_t usecs(uint64_t s);
uint64_t msecs(uint64_t s);
uint64_t seconds(uint64_t s);
uint64_t minutes(uint64_t s);
uint64_t hours(uint64_t s);

// Convert ticks to wall time units
float totalUsecs(uint64_t t);
float totalMsecs(uint64_t t);
float totalSeconds(uint64_t t);
float totalMinutes(uint64_t t);
float totalHours(uint64_t t);

// Delay functions
void time_spinFor(uint64_t duration);
void time_spinUntil(uint64_t then);

// Periodic macro execution helper
#define time_doPeriodic(period) \
for(uint64_t then = time_now() + (period); 1; then += (period), time_spinUntil(then))

// Human-readable time structure
typedef struct Time Time;
struct Time {
	uint16_t ticks;
	uint16_t msecs;
	uint8_t  seconds;
	uint8_t  minutes;
	uint16_t hours;
};

// Split / combine human-readable times
Time     time_split(uint64_t t);
uint64_t time_combine(Time t);
uint64_t ticksPerMs(void);

// Format macros for printing
#define time_fmtStr "Time(%uh, %um, %us, %ums, %ut)"
#define time_fmtParam(t) t.hours, t.minutes, t.seconds, t.msecs, t.ticks

// Call after SystemInit(); 1 ms SysTick, every 50 ms signals main to run PID.
void time_init(void);
bool time_take_control_tick(void);
