/*
 * joystick.h
 *
 * Created: 02.10.2026 10:44:19
 *  Author: quetzall
 */ 


#ifndef JOYSTICK_H_
#define JOYSTICK_H_

#include <stdint.h>

// Direcciones del Joystick
typedef enum {
	LEFT,
	RIGHT,
	UP,
	DOWN,
	NEUTRAL
} joystick_dir_t;

// Datos crudos leídos en una ráfaga del ADC
typedef struct {
	uint8_t joy_x;
	uint8_t joy_y;
	uint8_t slider_x;
	uint8_t slider_y;
} adc_raw_data_t;

// Estado procesado del Joystick
typedef struct {
	int8_t x;           // -100% a +100%
	int8_t y;           // -100% a +100%
	joystick_dir_t dir; // LEFT, RIGHT, UP, DOWN, NEUTRAL
} joystick_position_t;

// Estado procesado del Touchpad / Slider
typedef struct {
	uint8_t x;          // 0% a 100%
	uint8_t y;          // 0% a 100%
} slider_position_t;

// 2. Funciones de conversión / escalado
joystick_position_t joystick_process(adc_raw_data_t raw);
slider_position_t   slider_process(adc_raw_data_t raw);

#endif /* JOYSTICK_H_ */