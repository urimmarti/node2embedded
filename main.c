/* Node 2: CAN/servo/IR foreground work and dual-mode 20 ms PID controller.
 * SysTick only schedules updates; no floating point or motor operations in ISR.
 */
#include "sam.h"
#include "time.h"
#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>
#include <stdio.h>
#include "uart.h"
#include "can.h"
#include "PWM.h"
#include "ir.h"
#include "encoder.h"
#include "motor.h"
#include "position_controller.h"

#define F_CPU               84000000u
#define PIN_PC12_MASK       (1u << 12) // PC12
#define TOTAL_ENCODER_RANGE 5600
#define JOYSTICK_CENTER     164
#define JOYSTICK_MAX        85
#define CONTROL_PERIOD_MS   20         // Keep signed for negative velocity math
#define SWEEP_TIME_MS       700        // Keep signed for negative velocity math
#define CAN_TIMEOUT_MS      250u
#define SOLENOID_PULSE_MS   50u       // Active pulse duration on button press

// Joystick PID (smooth, sweep-rate-limited tracking)
#define JOY_PID_KP          0.06f
#define JOY_PID_KI          0.03f
#define JOY_PID_KD          0.0009f
#define JOY_PID_MIN_OUTPUT  (-50.0f)
#define JOY_PID_MAX_OUTPUT  50.0f

// Camera PID (fast response, ignores sweep time)
#define CAM_PID_KP          0.9f
#define CAM_PID_KI          0.03f
#define CAM_PID_KD          0.005f
#define CAM_PID_MIN_OUTPUT  (-30.0f)
#define CAM_PID_MAX_OUTPUT  30.0f

// Servo velocity-tilt parameters (5-pulse derivative when ball is near)
#define CAM_VEL_WINDOW      5          // Number of camera pulses for dx/dt window
#define SERVO_CENTER_US     1500
#define SERVO_MIN_US        900
#define SERVO_MAX_US        2100
#define CAM_VEL_DEADZONE    50.0f     // ticks/s: ignore stationary camera jitter
#define CAM_VEL_SERVO_GAIN  0.5      // us per (ticks/s): 4000 ticks/s -> full 600 us tilt

#define BALL_NEAR_Y_MAX         1500       // Always trigger BALL_NEAR below this Y
#define BALL_PREDICT_Y_MAX      2000       // Trigger early around Y ~ 1500 when approaching fast
#define BALL_APPROACH_VY_THRESH (-1200.0f) // ticks/s: negative = moving toward Y=0 goal

static void pc12_init(void) {
	PMC->PMC_PCER1 |= (1u << (ID_PIOC - 32));
	PIOC->PIO_PER  = PIN_PC12_MASK;
	PIOC->PIO_OER  = PIN_PC12_MASK;
	PIOC->PIO_CODR = PIN_PC12_MASK; // Start LOW
	PIOC->PIO_PUDR = PIN_PC12_MASK;
}

static int8_t decode_joystick(uint8_t raw) {
	int diff = (int)raw - JOYSTICK_CENTER;
	if (abs(diff) <= 3) return 0;
	if (diff > JOYSTICK_MAX) diff = JOYSTICK_MAX;
	if (diff < -JOYSTICK_MAX) diff = -JOYSTICK_MAX;
	return (int8_t)diff;
}

// Integrate the commanded velocity over a 20 ms sample (5600 ticks per full sweep).
static int32_t joystick_increment(int8_t input) {
	const int deadzone = 2;
	if (abs((int)input) <= deadzone) return 0;
	const int signed_magnitude = (input > 0) ? input - deadzone : input + deadzone;
	return (int32_t)signed_magnitude * (int32_t)TOTAL_ENCODER_RANGE * (int32_t)CONTROL_PERIOD_MS /
	((int32_t)(JOYSTICK_MAX - deadzone) * (int32_t)SWEEP_TIME_MS);
}

int main(void) {
	SystemInit();
	WDT->WDT_MR = WDT_MR_WDDIS;
	time_init();
	uart_init(F_CPU, 9600);
	printf("Node 2 ready\r\n");

	// 125 kbit/s: 84 MHz / (42 * 16 TQ). Match Node 1's MCP2515 at 16 MHz.
	can_init((CanInit){.brp = F_CPU / 2000000u - 1u,
	.phase1 = 5, .phase2 = 1, .propag = 6}, 0);
	encoder_init();
	ir_init();
	pc12_init();
	pwm_init();
	motor_init();
	motor_stop();

	PIDController joy_pid;
	PIDController cam_pid;
	pid_init(&joy_pid, JOY_PID_KP, JOY_PID_KI, JOY_PID_KD, JOY_PID_MIN_OUTPUT, JOY_PID_MAX_OUTPUT);
	pid_init(&cam_pid, CAM_PID_KP, CAM_PID_KI, CAM_PID_KD, CAM_PID_MIN_OUTPUT, CAM_PID_MAX_OUTPUT);

	int32_t target_position = encoder_read(); // Avoid a startup jump to zero.
	if (target_position < 0) target_position = 0;
	if (target_position > TOTAL_ENCODER_RANGE) target_position = TOTAL_ENCODER_RANGE;

	uint8_t raw_joystick = JOYSTICK_CENTER;
	int8_t joystick_pos = 0;
	int32_t joystick_inc = 0;
	int32_t current_servo_us = SERVO_CENTER_US;
	uint64_t last_can_time = time_now();

	uint64_t pc12_off_time = 0;
	bool pc12_active = false;
	bool button_was_pressed = false;
	bool ball_was_near = false;

	CanMsg rx_msg;

	uint16_t cam_x = 0xFFFF;
	uint16_t cam_y = 0xFFFF;
	int32_t mapped = -1;
	int32_t processed_y = -1;

	// 5-pulse ring buffer for 2D velocity (dx/dt and dy/dt)
	int32_t cam_x_hist[CAM_VEL_WINDOW] = {0};
	int32_t cam_y_hist[CAM_VEL_WINDOW] = {0};
	uint64_t cam_t_hist[CAM_VEL_WINDOW] = {0};
	uint8_t cam_hist_count = 0;
	uint8_t cam_hist_head = 0;
	float vel_x = 0.0f;
	float vel_y = 0.0f;

	bool camera_mode = true;
	bool prev_camera_mode = true;

	while (1) {
		// CAN is polled
		if (can_rx(&rx_msg) && rx_msg.id == 0) {
			// PROCESS CAMERA MESSAGE
			if (rx_msg.length == 4) {
				uint64_t rx_now = time_now();
				camera_mode = true;
				cam_x = ((uint16_t)rx_msg.byte[0] << 8) | rx_msg.byte[1];
				cam_y = ((uint16_t)rx_msg.byte[2] << 8) | rx_msg.byte[3];

				if (cam_x == 0xFFFF || cam_y == 0xFFFF) {
					mapped = -1;
					processed_y = -1;
					cam_hist_count = 0;
					cam_hist_head = 0;
					vel_x = 0.0f;
					vel_y = 0.0f;
					} else {
					// Reset history only if camera stalled for > 350 ms
					if (cam_hist_count > 0) {
						uint8_t last_idx = (cam_hist_head + CAM_VEL_WINDOW - 1) % CAM_VEL_WINDOW;
						if (rx_now - cam_t_hist[last_idx] > msecs(350)) {
							cam_hist_count = 0;
							cam_hist_head = 0;
							vel_x = 0.0f;
							vel_y = 0.0f;
						}
					}

					// Push current sample into the 5-pulse ring buffer
					cam_x_hist[cam_hist_head] = (int32_t)cam_x;
					cam_y_hist[cam_hist_head] = (int32_t)cam_y;
					cam_t_hist[cam_hist_head] = rx_now;
					cam_hist_head = (cam_hist_head + 1) % CAM_VEL_WINDOW;
					if (cam_hist_count < CAM_VEL_WINDOW) {
						cam_hist_count++;
					}

					// Compute derivatives across the buffered pulses (up to last 5)
					if (cam_hist_count >= 2) {
						uint8_t newest_idx = (cam_hist_head + CAM_VEL_WINDOW - 1) % CAM_VEL_WINDOW;
						uint8_t oldest_idx = (cam_hist_head + CAM_VEL_WINDOW - cam_hist_count) % CAM_VEL_WINDOW;
						uint64_t dt_ticks = cam_t_hist[newest_idx] - cam_t_hist[oldest_idx];

						if (dt_ticks > 0) {
							float dt_sec = (float)dt_ticks / (float)msecs(1000);
							vel_x = (float)(cam_x_hist[newest_idx] - cam_x_hist[oldest_idx]) / dt_sec;
							vel_y = (float)(cam_y_hist[newest_idx] - cam_y_hist[oldest_idx]) / dt_sec;
						}
					}

					uint16_t clamped_x = cam_x;
					if (clamped_x < 1000) clamped_x = 1000;
					if (clamped_x > 4600) clamped_x = 4600;

					// Maps [1000 .. 4600] (span = 3600) -> [0 .. 5600]
					mapped = ((int32_t)(clamped_x - 1000) * 5600L) / 3600L;
					processed_y = (int32_t)cam_y;
				}
			}

			// PROCESS JOYSTICK CAN MESSAGE
			if (rx_msg.length == 3) {
				camera_mode = false;
				raw_joystick = rx_msg.byte[0];
				joystick_pos = decode_joystick(raw_joystick);
				last_can_time = time_now();

				// Map input to servo pulse, with saturation before unsigned arithmetic.
				int32_t servo_us = 900 + ((int32_t)rx_msg.byte[1] - 67) * 1200 / 182;
				if (servo_us < SERVO_MIN_US) servo_us = SERVO_MIN_US;
				if (servo_us > SERVO_MAX_US) servo_us = SERVO_MAX_US;
				current_servo_us = servo_us;
				servo_set_duty_us((uint32_t)current_servo_us);

				// Trigger PC12 on button press edge (active LOW byte < 10)
				bool pressed = rx_msg.byte[2] < 10;
				if (pressed && !button_was_pressed && !pc12_active) {
					PIOC->PIO_SODR = PIN_PC12_MASK; // HIGH
					pc12_off_time = time_now() + msecs(SOLENOID_PULSE_MS);
					pc12_active = true;
				}
				button_was_pressed = pressed;
			}
		}

		// Reset newly activated PID and velocity history when switching modes
		if (camera_mode != prev_camera_mode) {
			if (camera_mode) {
				pid_reset(&cam_pid);
				cam_hist_count = 0;
				cam_hist_head = 0;
				vel_x = 0.0f;
				vel_y = 0.0f;
				} else {
				pid_reset(&joy_pid);
			}
			prev_camera_mode = camera_mode;
		}

		uint64_t now = time_now();

		// Automatically release PC12 after the pulse duration
		if (pc12_active && now >= pc12_off_time) {
			PIOC->PIO_CODR = PIN_PC12_MASK; // LOW
			pc12_active = false;
		}

		// Lost CAN must not leave a stale commanded velocity active indefinitely.
		if (now - last_can_time >= msecs(CAN_TIMEOUT_MS)) {
			joystick_pos = 0;
			raw_joystick = JOYSTICK_CENTER;
			button_was_pressed = false;
		}

		if (time_take_control_tick()) {
			int32_t current_position = encoder_read();
			int32_t output = 0;
			const float dt = (float)CONTROL_PERIOD_MS / 1000.0f;

			// -----------------------------------------------
			// JOYSTICK MODE (uses joy_pid + sweep time)
			// -----------------------------------------------
			if (!camera_mode) {
				joystick_inc = joystick_increment(joystick_pos);
				target_position += joystick_inc;

				if (target_position > TOTAL_ENCODER_RANGE) target_position = TOTAL_ENCODER_RANGE;
				if (target_position < 0) target_position = 0;

				output = pid_compute(&joy_pid, target_position, current_position, dt, false);
			}
			// ----------------------------------------------
			// CAMERA MODE (uses faster cam_pid, no sweep time)
			// ----------------------------------------------
			else {
				if (mapped >= 0) {
					target_position = mapped;
				}

				// Trigger when very close (< 1000) OR approaching rapidly around Y ~ 1500
				bool close_zone = (processed_y >= 0 && processed_y < BALL_NEAR_Y_MAX);
				bool fast_approach_zone = (processed_y >= 0 &&
				processed_y <= BALL_PREDICT_Y_MAX &&
				vel_y <= BALL_APPROACH_VY_THRESH);
				bool ball_near = close_zone || fast_approach_zone;

				if (ball_near) {
					if (!pc12_active) {
						if (fast_approach_zone && !close_zone) {
							printf("BALL_NEAR [FAST_Y] y:%ld | vy:%ld\r\n",
							(long)processed_y, (long)vel_y);
							} else {
							printf("BALL_NEAR [CLOSE] y:%ld | vy:%ld\r\n",
							(long)processed_y, (long)vel_y);
						}
						PIOC->PIO_SODR = PIN_PC12_MASK; // HIGH
						pc12_off_time = time_now() + msecs(SOLENOID_PULSE_MS);
						pc12_active = true;
					}

					// Tilt servo in the direction of the 5-pulse lateral velocity (dx/dt)
					int32_t servo_us = SERVO_CENTER_US;
					if (vel_x > CAM_VEL_DEADZONE) {
						servo_us = SERVO_CENTER_US + (int32_t)((vel_x - CAM_VEL_DEADZONE) * CAM_VEL_SERVO_GAIN);
						} else if (vel_x < -CAM_VEL_DEADZONE) {
						servo_us = SERVO_CENTER_US + (int32_t)((vel_x + CAM_VEL_DEADZONE) * CAM_VEL_SERVO_GAIN);
					}

					if (servo_us > SERVO_MAX_US) servo_us = SERVO_MAX_US;
					if (servo_us < SERVO_MIN_US) servo_us = SERVO_MIN_US;
					current_servo_us = servo_us;
					} else {
					current_servo_us = SERVO_CENTER_US;
				}
				servo_set_duty_us((uint32_t)current_servo_us);
				ball_was_near = ball_near;

				output = pid_compute(&cam_pid, target_position, current_position, dt, true);
			}

			motor_set(output);

			// Debug print every 100 ms (every 5th 20 ms tick)
			static uint8_t print_divider = 0;
			if (++print_divider >= 5) {
				print_divider = 0;
				if (camera_mode) {
					printf("[CAM] x:%u | y:%u | vx:%5ld | vy:%5ld | near:%d | srv:%4ldus | tgt:%4ld | Enc:%4ld | PID:%4ld\r\n",
					(unsigned)cam_x, (unsigned)cam_y,
					(long)vel_x, (long)vel_y, (int)ball_was_near, (long)current_servo_us,
					(long)target_position, (long)current_position, (long)output);
					} else {
					printf("[JOY] Raw:%3u | Dec:%4d | Step:%5ld | srv:%4ldus || [POS] Tgt:%4ld | Enc:%4ld | PID:%4ld\r\n",
					raw_joystick, (int)joystick_pos, (long)joystick_inc,
					(long)current_servo_us, (long)target_position,
					(long)current_position, (long)output);
				}
			}
		}

		if (ir_check_goal()) {
			printf("Goal! Score: %u\r\n", (unsigned)ir_get_score());
		}
	}
}