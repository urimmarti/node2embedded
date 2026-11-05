/*
 * motor.h
 *
 * Created: 30.10.2026 18:27:14
 *  Author: quetzall
 */ 


#ifndef MOTOR_H_
#define MOTOR_H_

void motor_init(void);

void motor_set(int percent);

void pwm_init_pb12(uint32_t frequency, uint8_t duty_percent);


void motor_stop(void);



#endif /* MOTOR_H_ */