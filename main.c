/*
 * node2.c
 * Author : quetzall
 */ 

#include "sam.h"
#include "time.h"
#include <stdint.h>
#include <stdio.h>
#include "uart.h"
#include "can.h"
#include "joystick.h"
#include "PWM.h"

#define F_CPU 84000000

void handle_joystick_servo(CanMsg *rx_msg) {
    if (rx_msg->id == 0) {
        // 1. Valor joystick_y viene en el índice correspondiente (ej. byte[1])
        uint8_t joystick_val = rx_msg->byte[1]; // Cambia a .data[1] si tu librería usa .data
        
        // 2. Mapear de [67, 255] a [900, 2100] microsegundos
        uint32_t target_us = (joystick_val - 67) * (2100 - 900) / (250 - 68) + 900;
		printf("test pwm %d\r\n", target_us);
		
		
        
        // 3. Actualizar el PWM del servo
        pwm_set_duty_us(target_us);
    }
}

int main(void) {
    SystemInit();
    
    WDT->WDT_MR = WDT_MR_WDDIS; // Desactivar watchdog

    uart_init(84000000, 9600);
    printf("Node 2 listo\r\n");
    
    can_init((CanInit){.brp = F_CPU/2000000-1, .phase1 = 5, .phase2 = 1, .propag = 6}, 0);
        
    pwm_init();
    
    uint8_t counter = 0;

    while (1) {
        // Enviar paquete periódico de prueba al Nodo 1 cada cierto tiempo (ej. bajando frecuencia)
        CanMsg tx_msg = {
            .id = 2,
            .length = 3,
            .byte = {10, 20, counter}
        };
        can_tx(tx_msg);
        counter++;

        // Recoger mensajes del bus CAN (como el joystick) de forma no bloqueante
        CanMsg rx_msg;
        if (can_rx(&rx_msg)) {
			printf("Received id %d, with byte[1] %d\r\n", rx_msg.id, rx_msg.byte[1]);
			handle_joystick_servo(&rx_msg);
        }

        // Espera de 50ms para no saturar
        time_spinFor(msecs(50));
    }
    
    return 0;
}