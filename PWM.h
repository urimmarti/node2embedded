/*
 * PWM.h
 *
 * Created: 27.10.2026 16:30:10
 *  Author: quetzall
 */ 


#ifndef PWM_H_
#define PWM_H_

#include <stdint.h>

void pwm_init();
void pwm_set_duty_us(uint32_t duty_us);

#endif /* PWM_H_ */