/* Node 2: CAN/servo/IR foreground work and a single 50 ms PID controller.
 * SysTick only schedules updates; no floating point or motor operations in ISR.
 */
#include "sam.h"
#include "time.h"
#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>
#include <stdio.h>
#include "uart.h"
#include "can.h"
#include "PWM.h"
#include "ir.h"
#include "encoder.h"
#include "motor.h"
#include "position_controller.h"

#define F_CPU               84000000u
#define PIN_PC12_MASK       (1u << 12) // PC12
#define TOTAL_ENCODER_RANGE 5600
#define JOYSTICK_CENTER     164
#define JOYSTICK_MAX        85
#define CONTROL_PERIOD_MS   50         // Keep signed for negative velocity math
#define SWEEP_TIME_MS       700       // Keep signed for negative velocity math
#define CAN_TIMEOUT_MS      250u
#define SOLENOID_PULSE_MS   100u       // Active pulse duration on button press

// Configure the PID here; it has exactly one owner (main), not a second ISR copy.
#define PID_KP 0.06f
#define PID_KI 0.01f
#define PID_KD 0.005f
#define PID_MIN_OUTPUT (-50.0f)
#define PID_MAX_OUTPUT 50.0f

static void pc12_init(void) {
    PMC->PMC_PCER1 |= (1u << (ID_PIOC - 32));
    PIOC->PIO_PER  = PIN_PC12_MASK;
    PIOC->PIO_OER  = PIN_PC12_MASK;
    PIOC->PIO_CODR = PIN_PC12_MASK; // Start LOW
    PIOC->PIO_PUDR = PIN_PC12_MASK;
}

static int8_t decode_joystick(uint8_t raw) {
    int diff = (int)raw - JOYSTICK_CENTER;
    if (abs(diff) <= 3) return 0;
    if (diff > JOYSTICK_MAX) diff = JOYSTICK_MAX;
    if (diff < -JOYSTICK_MAX) diff = -JOYSTICK_MAX;
    return (int8_t)diff;
}

// Integrate the commanded velocity over a 50 ms sample (5600 ticks per full sweep).
static int32_t joystick_increment(int8_t input) {
    const int deadzone = 2;
    if (abs((int)input) <= deadzone) return 0;
    const int signed_magnitude = (input > 0) ? input - deadzone : input + deadzone;
    return (int32_t)signed_magnitude * (int32_t)TOTAL_ENCODER_RANGE * (int32_t)CONTROL_PERIOD_MS /
           ((int32_t)(JOYSTICK_MAX - deadzone) * (int32_t)SWEEP_TIME_MS);
}

int main(void) {
    SystemInit();
    WDT->WDT_MR = WDT_MR_WDDIS;
    time_init();
    uart_init(F_CPU, 9600);
    printf("Node 2 ready\r\n");

    // 125 kbit/s: 84 MHz / (42 * 16 TQ). Match Node 1's MCP2515 at 16 MHz.
    can_init((CanInit){.brp = F_CPU / 2000000u - 1u,
                       .phase1 = 5, .phase2 = 1, .propag = 6}, 0);
    encoder_init();
    ir_init();
    pc12_init();
    pwm_init();
    motor_init();
    motor_stop();

    PIDController pos_pid;
    pid_init(&pos_pid, PID_KP, PID_KI, PID_KD, PID_MIN_OUTPUT, PID_MAX_OUTPUT);
    int32_t target_position = encoder_read(); // Avoid a startup jump to zero.
    if (target_position < 0) target_position = 0;
    if (target_position > TOTAL_ENCODER_RANGE) target_position = TOTAL_ENCODER_RANGE;
    
    uint8_t raw_joystick = JOYSTICK_CENTER;
    int8_t joystick_pos = 0;
    int32_t joystick_inc = 0;
    uint64_t last_can_time = time_now();

    uint64_t pc12_off_time = 0;
    bool pc12_active = false;
    bool button_was_pressed = false;

    CanMsg rx_msg;

    while (1) {
        // CAN is polled; expect at least 3 bytes (x-axis, y-axis, button)
        if (can_rx(&rx_msg) && rx_msg.id == 0 && rx_msg.length >= 3) {
            raw_joystick = rx_msg.byte[0];
            joystick_pos = decode_joystick(raw_joystick);
            last_can_time = time_now();

            // Map input to servo pulse, with saturation before unsigned arithmetic.
            int32_t servo_us = 900 + ((int32_t)rx_msg.byte[1] - 67) * 1200 / 182;
            if (servo_us < 900) servo_us = 900;
            if (servo_us > 2100) servo_us = 2100;
            servo_set_duty_us((uint32_t)servo_us);

            // Trigger PC12 on button press edge (active LOW byte < 10)
            bool pressed = rx_msg.byte[2] < 10;
            if (pressed && !button_was_pressed && !pc12_active) {
                PIOC->PIO_SODR = PIN_PC12_MASK; // HIGH
                pc12_off_time = time_now() + msecs(SOLENOID_PULSE_MS);
                pc12_active = true;
            }
            button_was_pressed = pressed;
        }

        uint64_t now = time_now();

        // Automatically release PC12 after the pulse duration
        if (pc12_active && now >= pc12_off_time) {
            PIOC->PIO_CODR = PIN_PC12_MASK; // LOW
            pc12_active = false;
        }

        // Lost CAN must not leave a stale commanded velocity active indefinitely.
        if (now - last_can_time >= msecs(CAN_TIMEOUT_MS)) {
            joystick_pos = 0;
            raw_joystick = JOYSTICK_CENTER;
            button_was_pressed = false;
        }

        if (time_take_control_tick()) {
            joystick_inc = joystick_increment(joystick_pos);
            target_position += joystick_inc;

            if (target_position > TOTAL_ENCODER_RANGE) target_position = TOTAL_ENCODER_RANGE;
            if (target_position < 0) target_position = 0;

            int32_t current_position = encoder_read();
            int32_t output = pid_compute(&pos_pid, target_position,
                                         current_position, (float)CONTROL_PERIOD_MS / 1000.0f);
            motor_set(output);

            // Debug print every 250 ms (every 5th tick)
            static uint8_t print_divider = 0;
            if (++print_divider >= 5) {
                print_divider = 0;
                /*printf("[JOY] Raw:%3u | Dec:%4d | Step:%5ld || [POS] Tgt:%4ld | Enc:%4ld | PID:%4ld\r\n",
                       raw_joystick, (int)joystick_pos, (long)joystick_inc,
                       (long)target_position, (long)current_position, (long)output);*/
            }
        }

        if (ir_check_goal()) {
            printf("Goal! Score: %u\r\n", (unsigned)ir_get_score());
        }
    }
}