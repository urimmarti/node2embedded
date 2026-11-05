#include "sam.h"
#include "encoder.h"

#define ENC_PHA_BIT (1u << 25)   // PC25 = TIOA6 (Fase A)
#define ENC_PHB_BIT (1u << 26)   // PC26 = TIOB6 (Fase B)[cite: 4]

void encoder_init(void) {
	// 1. Habilitar los relojes del puerto PIOC y del bloque TC2 (Canal 6 / TC6)[cite: 4]
	PMC->PMC_PCER0 |= (1u << ID_PIOC);
	PMC->PMC_PCER1 |= (1u << (ID_TC6 - 32));

	// 2. Configurar AMBOS pines (PC25 y PC26) para que usen la función de Periférico B[cite: 4]
	// (Esto conecta físicamente las señales A y B del encoder al bloque de hardware del temporizador)[cite: 4]
	PIOC->PIO_ABSR |= ENC_PHA_BIT | ENC_PHB_BIT;
	PIOC->PIO_PDR   = ENC_PHA_BIT | ENC_PHB_BIT; // Desactiva el control PIO clásico para dárselo al Periférico[cite: 4]

	// 3. Configurar el Block Mode Register (BMR) del bloque TC2 para activar el modo cuadratura completo[cite: 4]
	// QDEN: Habilita el QDEC[cite: 4]
	// POSEN: Habilita el modo de posición en el canal 0[cite: 4]
	// EDGPHA: Cuenta flancos de ambas fases (4x resolución)[cite: 4]
	// MAXFILT: Filtro para descartar pulsos de ruido rápido[cite: 4]
	TC2->TC_BMR = TC_BMR_QDEN | TC_BMR_POSEN | TC_BMR_EDGPHA | TC_BMR_MAXFILT(1);

	// 4. Configurar el reloj del canal y disparar el inicio del contador[cite: 4]
	TC2->TC_CHANNEL[0].TC_CMR = TC_CMR_TCCLKS_XC0;
	TC2->TC_CHANNEL[0].TC_CCR = TC_CCR_CLKEN | TC_CCR_SWTRG;
}

int32_t encoder_read(void) {
	// El registro Counter Value (TC_CV) almacena el conteo acumulado de posición con signo[cite: 4]
	return (int32_t)TC2->TC_CHANNEL[0].TC_CV;
}