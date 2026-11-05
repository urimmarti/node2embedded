/*
 * position_controller.h
 *
 * Created: 05.11.2026 12:42:37
 *  Author: quetzall
 */ 


#ifndef POSITION_CONTROLLER_H
#define POSITION_CONTROLLER_H

#include <stdint.h>
#include <stdbool.h>

#define MAX_POSITION 5600
#define MIN_POSITION 0
#define CENTER_POSITION 2800

typedef struct {
	float kp;
	float ki;
	float kd;
	float out_min;
	float out_max;
	float i_max;
	float integral;
	int32_t prev_error;
	int32_t deadband_ticks;
} PIDController;

void    pid_init(PIDController *pid, float kp, float ki, float kd, float out_min, float out_max);
void    pid_reset(PIDController *pid);
int32_t map_joystick_to_setpoint(uint8_t joystick_val);
int32_t pid_compute(PIDController *pid, int32_t setpoint, int32_t current_pos, float dt);


#endif /* POSITION_CONTROLLER_H_ */