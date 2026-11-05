#ifndef ENCODER_H
#define ENCODER_H

#include <stdint.h>



// Inicializa el bloque TC2 en modo decodificador en cuadratura
void encoder_init(void);

// Lee el valor actual acumulado de los pulsos del encoder (posición relativa)
int32_t encoder_read(void);

#endif