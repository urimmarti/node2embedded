/*
 * joystick.c
 *
 * Created: 02.10.2026 10:44:06
 *  Author: quetzall
 */ 

#include "joystick.h"
#include <stdlib.h>

// Calibración Joystick
#define JOY_RAW_MIN   65   // ~1.0V -> -100%
#define JOY_RAW_MID  160   // Tu centro calibrado por software -> 0%
#define JOY_RAW_MAX  250   // ~4.0V -> +100%

// Calibración Slider
#define SLIDER_RAW_MIN   0
#define SLIDER_RAW_MAX 255


// --- CONVERSIONES Y ESCALADOS ---

static int8_t scale_joystick(uint8_t raw) {
	// Deadzone en reposo alrededor de 160
	if (raw >= (JOY_RAW_MID - 3) && raw <= (JOY_RAW_MID + 3)) {
		return 0;
	}
	
	if (raw <= JOY_RAW_MIN) return -100;
	if (raw >= JOY_RAW_MAX) return 100;

	if (raw < JOY_RAW_MID) {
		return (int8_t)(((int16_t)(raw - JOY_RAW_MID) * 100) / (JOY_RAW_MID - JOY_RAW_MIN));
		} else {
		return (int8_t)(((int16_t)(raw - JOY_RAW_MID) * 100) / (JOY_RAW_MAX - JOY_RAW_MID));
	}
}

static uint8_t scale_slider(uint8_t raw) {
	if (raw <= SLIDER_RAW_MIN) return 0;
	if (raw >= SLIDER_RAW_MAX) return 100;

	return (uint8_t)(((uint16_t)raw * 100) / SLIDER_RAW_MAX);
}

static joystick_dir_t calculate_direction(int8_t x, int8_t y) {
	uint8_t threshold = 50;

	if (abs(x) > abs(y)) {
		if (x > threshold)  return RIGHT;
		if (x < -threshold) return LEFT;
		} else {
		if (y > threshold)  return UP;
		if (y < -threshold) return DOWN;
	}

	return NEUTRAL;
}

joystick_position_t joystick_process(adc_raw_data_t raw) {
	joystick_position_t joy;
	joy.x = scale_joystick(raw.joy_x);
	joy.y = scale_joystick(raw.joy_y);
	joy.dir = calculate_direction(joy.x, joy.y);
	return joy;
}

slider_position_t slider_process(adc_raw_data_t raw) {
	slider_position_t slider;
	slider.x = scale_slider(raw.slider_x);
	slider.y = scale_slider(raw.slider_y);
	return slider;
}