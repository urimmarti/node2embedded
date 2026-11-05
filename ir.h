/*
 * ir.h
 *
 * Created: 29.10.2026 15:29:48
 *  Author: quetzall
 */ 


#ifndef IR_H
#define IR_H

#include <stdint.h>
#include <stdbool.h>

// Inicializa el ADC y el canal 0 para el sensor IR
void ir_init(void);

// Lee el valor bruto del ADC (0 - 4095)
uint32_t ir_read_raw(void);

// Comprueba si se ha detectado un gol (incluye umbral y antirrebote)
bool ir_check_goal(void);

// Devuelve la puntuación actual acumulada
uint8_t ir_get_score(void);

#endif