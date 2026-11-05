/*
 * node2.c
 * Author : quetzall
 */ 

#include "sam.h"
#include "time.h"
#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>
#include <stdio.h>
#include "uart.h"
#include "can.h"
#include "joystick.h"
#include "PWM.h"
#include "ir.h"
#include "encoder.h"
#include "motor.h"
#include "position_controller.h"

#define F_CPU 84000000
#define PIN_RELAY_MASK (1 << 12) // PC12 (Pin D51)
#define TOTAL_ENCODER_RANGE 5600
#define JOYSTICK_MAX        85

void solenoid_init(void) {
    PMC->PMC_PCER1 |= (1 << (ID_PIOC - 32));
    PIOC->PIO_PER = PIN_RELAY_MASK;
    PIOC->PIO_OER = PIN_RELAY_MASK;
    PIOC->PIO_PUDR = PIN_RELAY_MASK;
}

void solenoid_trigger(void) {
    PIOC->PIO_SODR = PIN_RELAY_MASK;
    time_spinFor(msecs(100));
    PIOC->PIO_CODR = PIN_RELAY_MASK;
}

int32_t get_joystick_increment(int8_t joystick_val, uint32_t sweep_time_ms, uint32_t dt_ms) {
    const int32_t DEADZONE = 2;

    if (sweep_time_ms == 0 || dt_ms == 0) {
        return 0;
    }

    // Clamp input limits
    if (joystick_val > JOYSTICK_MAX)  joystick_val = JOYSTICK_MAX;
    if (joystick_val < -JOYSTICK_MAX) joystick_val = -JOYSTICK_MAX;

    // Deadzone check
    if (abs(joystick_val) <= DEADZONE) {
        return 0;
    }

    int32_t effective_joy;
    int32_t max_joy_range = JOYSTICK_MAX - DEADZONE;

    if (joystick_val > 0) {
        effective_joy = (int32_t)joystick_val - DEADZONE;
    } else {
        effective_joy = (int32_t)joystick_val + DEADZONE;
    }

    // Multiply before dividing to prevent rounding to 0
    int32_t numerator   = effective_joy * TOTAL_ENCODER_RANGE * (int32_t)dt_ms;
    int32_t denominator = max_joy_range * (int32_t)sweep_time_ms;

    return numerator / denominator;
}

int main(void) {
    SystemInit();
    
    WDT->WDT_MR = WDT_MR_WDDIS; // Desactivar watchdog

    uart_init(84000000, 9600);
    printf("Node 2 listo\r\n");
    
    can_init((CanInit){.brp = F_CPU/2000000-1, .phase1 = 5, .phase2 = 1, .propag = 6}, 0);
      
    encoder_init();
	ir_init();
    solenoid_init();
    pwm_init();
    motor_init();

    // 2. PID Controller Initialization
    PIDController pos_pid;
    // Kp = 0.08, Ki = 0.01, Kd = 0.005, output limits [-50, 50]
    pid_init(&pos_pid, 0.08f, 0.01f, 0.005f, -50.0f, 50.0f);

    // Persistent Control Variables
    int8_t   joystick_pos = 0;       // Dynamic signed joystick offset [-85..+85]
    int32_t  target_position = 0;    // Dynamic accumulated setpoint [0..5600]
    uint64_t last_pid_time = time_now();
    CanMsg   rx_msg;

    const uint32_t SWEEP_TIME_MS = 1000; // 1 second full range
    const uint32_t DT_MS         = 50;   // 50 ms sample rate
	uint8_t joystick_x;

	while (1) {
		// -------------------------------------------------------------
		// CAN Message Processing
		// -------------------------------------------------------------
		if (can_rx(&rx_msg)) {
			if (rx_msg.id == 0) {
				// Update joystick raw reading & map center 164 -> signed [-85..+85]
				uint8_t raw_x = rx_msg.byte[0];
				int diff = (int)raw_x - 164;
				int8_t joy_pos = 0;

				if (abs(diff) > 3) {
					if (diff > 85)  diff = 85;
					if (diff < -85) diff = -85;
					joy_pos = (int8_t)diff;
				}

				// Pass updated joystick state directly to background SysTick PID loop
				time_set_joystick_input(joy_pos);

				// Update Servo Position: Map [67..255] -> [900..2100] us
				uint8_t joystick_y = rx_msg.byte[1];
				uint32_t servo_us = (uint32_t)((int32_t)joystick_y - 67) * (2100 - 900) / (250 - 68) + 900;
				servo_set_duty_us(servo_us);

				// Update Solenoid Output
				uint8_t button_state = rx_msg.byte[2];
				if (button_state < 10) {
					solenoid_trigger();
				}
			}
		}

		if (ir_check_goal()) {
			uint8_t current_score = ir_get_score();
			printf("¡GOL! Puntuacion actual: %d\r\n", current_score);
		}
	}


    while (1) {
        // -------------------------------------------------------------
        // Task A: Process Incoming CAN Messages (Runs as fast as possible)
        // -------------------------------------------------------------
        if (can_rx(&rx_msg)) {
            if (rx_msg.id == 0) {
                // Update joystick raw reading & map center 164 -> signed [-85..+85]
                joystick_x = rx_msg.byte[0];
                int diff = (int)joystick_x - 164;
                if (abs(diff) <= 3) {
                    joystick_pos = 0;
                } else {
                    if (diff > JOYSTICK_MAX)  diff = JOYSTICK_MAX;
                    if (diff < -JOYSTICK_MAX) diff = -JOYSTICK_MAX;
                    joystick_pos = (int8_t)diff;
                }

                // Update Servo Position: Map [67..255] -> [900..2100] us
                uint8_t joystick_y = rx_msg.byte[1];
                uint32_t servo_us = (uint32_t)((int32_t)joystick_y - 67) * (2100 - 900) / (250 - 68) + 900;
                servo_set_duty_us(servo_us);

                // Update Solenoid Output
                uint8_t button_state = rx_msg.byte[2];
                if (button_state < 10) {
                    solenoid_trigger();
                }
            }
        }

        if (ir_check_goal()) {
            uint8_t current_score = ir_get_score();
            printf("¡GOL! Puntuacion actual: %d\r\n", current_score);
        }

        // -------------------------------------------------------------
        // Task B: Non-Blocking 50 ms Periodic PID Position Control Loop
        // -------------------------------------------------------------
        if ((time_now() - last_pid_time) >= msecs(DT_MS)) {
            last_pid_time = time_now();
            const float DT = 0.050f;

            // 1. Calculate step increase from signed joystick position
            int32_t increase = get_joystick_increment(joystick_pos, SWEEP_TIME_MS, DT_MS);

            // 2. Accumulate position target and clamp [0..5600]
            target_position += increase;
            if (target_position > TOTAL_ENCODER_RANGE) target_position = TOTAL_ENCODER_RANGE;
            if (target_position < 0)                   target_position = 0;

            // 3. Read encoder feedback
            int32_t current_pos = encoder_read();

            // 4. Compute PID output command using accumulated target_position
            int32_t motor_speed = pid_compute(&pos_pid, target_position, current_pos, DT);

            // 5. Update motor driver hardware
            //motor_set(motor_speed);

				  
				  
			// 1. Map joystick position [80..250] to target setpoint [0..5600]

			int32_t setpoint = map_joystick_to_setpoint(joystick_x);


			current_pos = encoder_read();
			motor_speed = pid_compute(&pos_pid, setpoint, current_pos, DT);
			motor_set(motor_speed);
			          //  printf("motor speed %d | joystick_pos %d | pos_increase %d | desired_pos %d | enc %d\r\n",
			          //  motor_speed, joystick_pos, increase, target_position, current_pos);
        }
    }
    
    return 0;
}