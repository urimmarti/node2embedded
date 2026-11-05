/*
 * position_controller.c
 *
 * Created: 05.11.2026 12:42:54
 *  Author: quetzall
 */ 
#include "position_controller.h"
#include <stdlib.h>

#define SWEEP_TIME 1000
#define SAMPLE_RATE 20
#define MAX_JOYSTICK_VAL 85
#define ENCODER_RANGE 5600


void pid_init(PIDController *pid, float kp, float ki, float kd, float out_min, float out_max) {
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    pid->out_min = out_min;
    pid->out_max = out_max;
    pid->i_max = out_max * 0.5f; // Clamp integral term to 50% of maximum output
    pid->deadband_ticks = 4;      // Ignore tiny errors (< 4 ticks) to avoid motor chatter
    pid_reset(pid);
}

void pid_reset(PIDController *pid) {
    pid->integral   = 0.0f;
    pid->prev_error = 0;
}

/**
 * @brief Maps joystick raw value [80..250] to target position [0..5600]
 *        Center (164) maps directly to 2800.
 */
int32_t map_joystick_to_setpoint(uint8_t joystick_val) {
    const uint8_t CENTER   = 164;
    const uint8_t DEADZONE = 3;

    int delta = (int)joystick_val - CENTER;

    // Apply center deadzone
    if (abs(delta) <= DEADZONE) {
        return CENTER_POSITION;
    }

    if (joystick_val > CENTER) {
        // Range [164..250] -> [2800..5600]
        return CENTER_POSITION + (int32_t)(joystick_val - CENTER) * (MAX_POSITION - CENTER_POSITION) / (250 - CENTER);
    } else {
        // Range [80..164] -> [0..2800]
        return CENTER_POSITION - (int32_t)(CENTER - joystick_val) * CENTER_POSITION / (CENTER - 80);
    }
}

/**
 * @brief PID Calculation: Output = Kp*e + Ki*?e dt + Kd*(de/dt)
 */
int32_t pid_compute(PIDController *pid, int32_t setpoint, int32_t current_pos, float dt) {
    if (dt <= 0.0f) return 0;

    int32_t error = setpoint - current_pos;

    // 1. Deadband check
    if (abs(error) <= pid->deadband_ticks) {
        pid->prev_error = error;
        return 0;
    }

    // 2. Proportional Term
    float p_term = pid->kp * (float)error;

    // 3. Integral Term with Anti-Windup Clamping
    pid->integral += (float)error * dt;
    float i_term = pid->ki * pid->integral;

    if (i_term > pid->i_max) {
        i_term = pid->i_max;
        pid->integral = pid->i_max / pid->ki;
    } else if (i_term < -pid->i_max) {
        i_term = -pid->i_max;
        pid->integral = -pid->i_max / pid->ki;
    }

    // 4. Derivative Term
    float derivative = ((float)error - (float)pid->prev_error) / dt;
    float d_term = pid->kd * derivative;

    pid->prev_error = error;

    // 5. Total Output with Saturation Limits
    float output = p_term + i_term + d_term;

    if (output > pid->out_max) output = pid->out_max;
    if (output < pid->out_min) output = pid->out_min;

    return (int32_t)output;
}

int map_joystick_to_pos_increase(int joystick_pos){
	//with the joystick set at max side, in sweep_time ms there must have been a 0-5600 increase
	int max_pos_increase_per_sample = ENCODER_RANGE / (SWEEP_TIME);
	// DO A MAPPING: the range 0-85 , must go to the range 0-MAX_INCREASE
	return joystick_pos *max_pos_increase_per_sample / MAX_JOYSTICK_VAL;

}
int update_desired_pos(int desired_pos, int increase){
	desired_pos+=increase;
	if (desired_pos <1){
		desired_pos = 1;
	}
	if (desired_pos > 5614){
		desired_pos = 5610;
	}
	return desired_pos;
}

