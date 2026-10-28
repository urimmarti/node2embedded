/*
 * PWM.c
 *
 * Created: 27.10.2026 16:29:58
 *  Author: quetzall
 */ 

#include "sam.h"
#include "PWM.h"

#define PWM_CHANNEL_ID 1   // PC21 = PWML4, peripheral B

void pwm_init(void) {
	PMC->PMC_PCER1 = (1u << (ID_PWM - 32));   // PWM clock
	PMC->PMC_PCER0 = (1u << ID_PIOB);         // PIOC clock

	PIOB->PIO_ABSR |= (1u << 13);             // peripheral B
	PIOB->PIO_PDR   = (1u << 13);             // hand pin to peripheral

	PWM->PWM_CLK = PWM_CLK_PREA(0) | PWM_CLK_DIVA(84);   // CLKA = 1 MHz

	PWM->PWM_DIS = (1u << PWM_CHANNEL_ID);
	PWM->PWM_CH_NUM[PWM_CHANNEL_ID].PWM_CMR  = PWM_CMR_CPRE_CLKA | PWM_CMR_CPOL;
	PWM->PWM_CH_NUM[PWM_CHANNEL_ID].PWM_CPRD = 20000;    // 20 ms
	PWM->PWM_CH_NUM[PWM_CHANNEL_ID].PWM_CDTY = 1500;     // 1.5 ms
	PWM->PWM_ENA = (1u << PWM_CHANNEL_ID);
}

void pwm_set_duty_us(uint32_t duty_us) {
	if (duty_us < 900)  duty_us = 900;
	if (duty_us > 2100) duty_us = 2100;
	PWM->PWM_CH_NUM[PWM_CHANNEL_ID].PWM_CDTYUPD = duty_us;
}
