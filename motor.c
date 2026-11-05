#include "sam.h"
#include "motor.h"

// ---- PINES: VERIFICA CON EL "TTK4155 Motor Shield document" / esquema del shield ----
// Valores de ejemplo. PB12 es PWMH0 (periferico B) segun el datasheet SAM3X,
// pero que el shield use ese pin hay que confirmarlo en su documento.
#define MOTOR_PWM_CH      0
#define MOTOR_PWM_PIO     PIOB
#define MOTOR_PWM_BIT     (1u << 12)   // PB12 = PWMH0, periferico B
#define MOTOR_DIR_PIO     PIOC
#define MOTOR_DIR_BIT     (1u << 23)   // EJEMPLO: pin de DIRECCION (entrada de la NOR)
#define MOTOR_DIR_INVERT  0            // pon 1 si el sentido sale invertido

// Polaridad de la senal PWM tras la NOR. 0 = PWM en alto => motor excitado (lo normal que
// se espera). 1 = PWM en bajo => motor excitado. NO LO SE: depende del cableado de la NOR.
// Compruebalo ANTES de dar 12 V: con duty 0 las entradas IN1 e IN2 del DRV8871 deben
// estar AMBAS en bajo (motor parado). Si ves el motor excitado con duty 0, pon 1.
#define MOTOR_PWM_ACTIVE_LOW 0

// El DRV8871 exige >= 800 ns de ancho de pulso para detectar la entrada (datasheet 6.3, nota 1).
// A 20 kHz eso es ~1.6 %; por debajo de 2 % lo tratamos como 0 para no tener duty "fantasma".
#define MOTOR_MIN_PERCENT 2

#define MOTOR_PWM_PERIOD  4200u        // 84 MHz / 4200 = 20 kHz (inaudible)
#define MOTOR_MAX_PERCENT 80           // tope de seguridad

void motor_init(void){
	PMC->PMC_PCER0 |= (1u << ID_PIOB) | (1u << ID_PIOC);
	PMC->PMC_PCER1 |= (1u << (ID_PWM - 32));

	// Pin de PWM -> periferico B
	MOTOR_PWM_PIO->PIO_ABSR |= MOTOR_PWM_BIT;
	MOTOR_PWM_PIO->PIO_PDR   = MOTOR_PWM_BIT;

	// Pin de sentido -> GPIO de salida
	MOTOR_DIR_PIO->PIO_PER  = MOTOR_DIR_BIT;
	MOTOR_DIR_PIO->PIO_OER  = MOTOR_DIR_BIT;
	MOTOR_DIR_PIO->PIO_CODR = MOTOR_DIR_BIT;

	// Canal propio, reloj = MCK directo (no toca CLKA, que usa el servo).
	// Activo en alto: CPOL = 1 -> CDTY es el tiempo en alto; CDTY = 0 deja la salida en bajo (parado).
	// Activo en bajo: CPOL = 0 -> CDTY es el tiempo en bajo; CDTY = 0 deja la salida en alto (parado).
	PWM->PWM_DIS = (1u << MOTOR_PWM_CH);
	PWM->PWM_CH_NUM[MOTOR_PWM_CH].PWM_CMR  = PWM_CMR_CPRE_MCK | (MOTOR_PWM_ACTIVE_LOW ? 0u : PWM_CMR_CPOL);
	PWM->PWM_CH_NUM[MOTOR_PWM_CH].PWM_CPRD = MOTOR_PWM_PERIOD;
	PWM->PWM_CH_NUM[MOTOR_PWM_CH].PWM_CDTY = 0;
	PWM->PWM_ENA = (1u << MOTOR_PWM_CH);
}

#define PWM_CLOCK 1000000UL   // 1 MHz internal PWM clock

void pwm_init_pb12(uint32_t frequency, uint8_t duty_percent)
{
    /* 1. Enable peripheral clock for PWM controller (ID 36) */
    PMC->PMC_PCER1 |= PMC_PCER1_PID36;

    /* 2. PB12 must be controlled by Peripheral B = PWMH0 */
    PIOB->PIO_ABSR |= PIO_ABSR_P12;   // Peripheral B
    PIOB->PIO_PDR  |= PIO_PDR_P12;    // Disable GPIO control

    /* Stop channel 0 while configuring */
    PWM->PWM_DIS = PWM_DIS_CHID0;

    /*
     * 3. Create CLKA = 84 MHz / 84 = 1 MHz
     *
     * PREA = 0 -> no 2^n prescaling
     * DIVA = 84
     */
    PWM->PWM_CLK =
        PWM_CLK_PREA(0) |
        PWM_CLK_DIVA(84);

    /*
     * 4. Channel 0:
     *    - use CLKA
     *    - left-aligned PWM
     *    - CPOL = 1 -> output starts HIGH
     */
    PWM->PWM_CH_NUM[0].PWM_CMR =
        PWM_CMR_CPRE_CLKA |
        PWM_CMR_CPOL;

    /* Number of PWM clock ticks per period */
    uint32_t period = PWM_CLOCK / frequency;

    /* Protect against invalid values */
    if (period < 1)
        period = 1;

    if (period > 65535)
        period = 65535;

    if (duty_percent > 100)
        duty_percent = 100;

    PWM->PWM_CH_NUM[0].PWM_CPRD = period;

    /* High-time */
    PWM->PWM_CH_NUM[0].PWM_CDTY =
        (period * duty_percent) / 100;

    /* 5. Enable PWM channel 0 */
    PWM->PWM_ENA = PWM_ENA_CHID0;
}


void motor_set(int percent){
	int forward = (percent >= 0);
	if(percent < 0) percent = -percent;
	if(percent > MOTOR_MAX_PERCENT) percent = MOTOR_MAX_PERCENT;
	if(percent < MOTOR_MIN_PERCENT) percent = 0;

	if(forward ^ MOTOR_DIR_INVERT){
		MOTOR_DIR_PIO->PIO_CODR = MOTOR_DIR_BIT;
		} else {
		MOTOR_DIR_PIO->PIO_SODR = MOTOR_DIR_BIT;
	}
	PWM->PWM_CH_NUM[MOTOR_PWM_CH].PWM_CDTYUPD = (MOTOR_PWM_PERIOD * (uint32_t)percent) / 100u;
}

void motor_stop(void){
	PWM->PWM_CH_NUM[MOTOR_PWM_CH].PWM_CDTYUPD = 0;
}

