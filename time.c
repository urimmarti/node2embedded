#include <stddef.h>
#include <stdint.h>
#include <stdlib.h>
#include <stdbool.h>
#include <sam.h>

#include "time.h"
#include "encoder.h"
#include "position_controller.h"

#define TOTAL_ENCODER_RANGE 5600
#define JOYSTICK_MAX        85

extern void motor_set(int speed);

// Timing variables
uint64_t calib;
static uint64_t now = 0;

// SysTick PID control loop state
static PIDController pos_pid;
static volatile int8_t  g_joystick_pos = 0;     // Signed input [-85..+85]
static volatile int32_t g_target_position = 0;  // Accumulated setpoint [0..5600]
static uint32_t ms_counter = 0;

static int32_t get_joystick_increment(int8_t joystick_val, uint32_t sweep_time_ms, uint32_t dt_ms) {
	const int32_t DEADZONE = 2;

	if (sweep_time_ms == 0 || dt_ms == 0) {
		return 0;
	}

	if (joystick_val > JOYSTICK_MAX)  joystick_val = JOYSTICK_MAX;
	if (joystick_val < -JOYSTICK_MAX) joystick_val = -JOYSTICK_MAX;

	if (abs(joystick_val) <= DEADZONE) {
		return 0;
	}

	int32_t effective_joy;
	int32_t max_joy_range = JOYSTICK_MAX - DEADZONE;

	if (joystick_val > 0) {
		effective_joy = (int32_t)joystick_val - DEADZONE;
		} else {
		effective_joy = (int32_t)joystick_val + DEADZONE;
	}

	int32_t numerator   = effective_joy * TOTAL_ENCODER_RANGE * (int32_t)dt_ms;
	int32_t denominator = max_joy_range * (int32_t)sweep_time_ms;

	return numerator / denominator;
}

__attribute__((constructor)) void time_init(void) {
	calib = SysTick->CALIB * 8;
	SysTick->LOAD = (calib & SysTick_LOAD_RELOAD_Msk) - 1;
	SysTick->VAL = 0;
	NVIC_SetPriority(SysTick_IRQn, 2);

	SysTick->CTRL =
	((1 << SysTick_CTRL_CLKSOURCE_Pos) & SysTick_CTRL_CLKSOURCE_Msk) |
	((1 << SysTick_CTRL_TICKINT_Pos)   & SysTick_CTRL_TICKINT_Msk)   |
	((1 << SysTick_CTRL_ENABLE_Pos)    & SysTick_CTRL_ENABLE_Msk);

	// Initialize PID Controller (Kp = 0.08, Ki = 0.01, Kd = 0.005, output limits [-50, 50])
	pid_init(&pos_pid, 0.08f, 0.01f, 0.000f, -50.0f, 50.0f);
}

void SysTick_Handler(void) {
	now += calib;
	ms_counter++;

	// Deterministic 50 ms Control Loop Execution
	if (ms_counter >= 50) {
		ms_counter = 0;

		const uint32_t SWEEP_TIME_MS = 1000;
		const uint32_t DT_MS         = 50;
		const float    DT            = 0.050f;

		// 1. Calculate rate-integrated setpoint step
		int32_t increase = get_joystick_increment(g_joystick_pos, SWEEP_TIME_MS, DT_MS);

		// 2. Accumulate target position and clamp [0..5600]
		int32_t target = g_target_position + increase;
		if (target > TOTAL_ENCODER_RANGE) target = TOTAL_ENCODER_RANGE;
		if (target < 0)                   target = 0;
		g_target_position = target;

		// 3. Read encoder feedback
		int32_t current_pos = encoder_read();

		// 4. Compute PID command
		int32_t motor_speed = pid_compute(&pos_pid, g_target_position, current_pos, DT);

		// 5. Drive motor output
		motor_set(motor_speed);
	}
}

void time_set_joystick_input(int8_t joystick_val) {
	g_joystick_pos = joystick_val;
}

int32_t time_get_target_position(void) {
	return g_target_position;
}

uint64_t time_now(void) {
	return now + calib - SysTick->VAL;
}

uint64_t usecs(uint64_t s)   { return s * calib / 1000; }
uint64_t msecs(uint64_t s)   { return s * calib; }
uint64_t seconds(uint64_t s) { return s * 1000 * calib; }
uint64_t minutes(uint64_t s) { return s * 60 * 1000 * calib; }
uint64_t hours(uint64_t s)   { return s * 60 * 60 * 1000 * calib; }

float totalUsecs(uint64_t t)   { return t / (1.0 * calib / 1000); }
float totalMsecs(uint64_t t)   { return t / (1.0 * calib); }
float totalSeconds(uint64_t t) { return t / (1.0 * calib * 1000); }
float totalMinutes(uint64_t t) { return t / (1.0 * calib * 1000 * 60); }
float totalHours(uint64_t t)   { return t / (1.0 * calib * 1000 * 60 * 60); }

Time time_split(uint64_t t) {
	Time r;
	uint64_t d;

	d = t / (calib * 1000 * 60 * 60);
	r.hours = (uint16_t)d;
	t -= d * (calib * 1000 * 60 * 60);
	
	d = t / (calib * 1000 * 60);
	r.minutes = (uint8_t)d;
	t -= d * (calib * 1000 * 60);
	
	d = t / (calib * 1000);
	r.seconds = (uint8_t)d;
	t -= d * (calib * 1000);
	
	d = t / calib;
	r.msecs = (uint16_t)d;
	r.ticks = (uint16_t)(t - d * calib);

	return r;
}

uint64_t time_combine(Time t) {
	return t.ticks + msecs(t.msecs) + seconds(t.seconds) + minutes(t.minutes) + hours(t.hours);
}

void time_spinFor(uint64_t duration) {
	time_spinUntil(time_now() + duration);
}

void time_spinUntil(uint64_t then) {
	while (then > time_now()) {}
}

uint64_t ticksPerMs(void) {
	return calib;
}