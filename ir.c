/*
 * ir.c
 *
 * Created: 29.10.2026 15:29:38
 *  Author: quetzall
 */ 


#include "sam.h"
#include "ir.h"
#include "time.h"

// Ajusta este umbral según lo que veas por la consola al pasar la pelota
#define IR_THRESHOLD 2000

static uint8_t score = 0;
static bool was_blocked = false;

void ir_init(void) {
	// 1. Habilitar reloj del ADC en el PMC (ID_ADC = 37 -> PMC_PCER1)
	PMC->PMC_PCER1 |= (1 << (ID_ADC - 32));

	// 2. Resetear el ADC
	ADC->ADC_CR = ADC_CR_SWRST;

	// 3. Configurar modo (trigger por software, prescaler estándar)
	ADC->ADC_MR = ADC_MR_PRESCAL(2) | ADC_MR_STARTUP_SUT8;

	// 4. Habilitar el Canal 0 (ADC0 / Pin PA2 en la placa de expansión)
	ADC->ADC_CHER = (1 << 0);
}

uint32_t ir_read_raw(void) {
	// Iniciar conversión por software
	ADC->ADC_CR = ADC_CR_START;

	// Esperar a que termine la conversión del canal 0 (EOC0)
	while (!(ADC->ADC_ISR & (1 << 0)));

	// Devolver el resultado de 12 bits
	return ADC->ADC_CDR[0];
}

bool ir_check_goal(void) {
	uint32_t current_val = ir_read_raw();

	// Detección por umbral (si el haz se interrumpe y el voltaje cae/sube según tu circuito)
	// Asumimos que cae por debajo del umbral al bloquearse el haz:
	bool currently_blocked = (current_val < IR_THRESHOLD);

	// Lógica anti-rebote (Edge detection: solo cuenta en el flanco de bajada de la luz)
	if (currently_blocked && !was_blocked) {
		was_blocked = true;
		score++;
		return true; // ¡Gol registrado!
	}
	
	// Reseteamos el estado cuando la pelota sale y vuelve la luz
	if (!currently_blocked) {
		was_blocked = false;
	}

	return false;
}

uint8_t ir_get_score(void) {
	return score;
}