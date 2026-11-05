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

#define F_CPU                 84000000u
#define PIN_RELAY_MASK        (1u << 12) // PC12
#define TOTAL_ENCODER_RANGE  5600
#define JOYSTICK_CENTER      164
#define JOYSTICK_MAX         85
#define CONTROL_PERIOD_MS    50u
#define SWEEP_TIME_MS        1000u
#define CAN_TIMEOUT_MS       250u
#define SOLENOID_PULSE_MS    100u

// Configure the PID here; it has exactly one owner (main), not a second ISR copy.
#define PID_KP 0.08f
#define PID_KI 0.01f
#define PID_KD 0.005f
#define PID_MIN_OUTPUT (-50.0f)
#define PID_MAX_OUTPUT 50.0f

static void solenoid_init(void) {
    PMC->PMC_PCER1 |= (1u << (ID_PIOC - 32));
    PIOC->PIO_PER = PIN_RELAY_MASK;
    PIOC->PIO_OER = PIN_RELAY_MASK;
    PIOC->PIO_CODR = PIN_RELAY_MASK;
    PIOC->PIO_PUDR = PIN_RELAY_MASK;
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
    return (int32_t)signed_magnitude * TOTAL_ENCODER_RANGE * CONTROL_PERIOD_MS /
           ((JOYSTICK_MAX - deadzone) * SWEEP_TIME_MS);
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
    solenoid_init();
    pwm_init();
    motor_init();
    motor_stop();

    PIDController pos_pid;
    pid_init(&pos_pid, PID_KP, PID_KI, PID_KD, PID_MIN_OUTPUT, PID_MAX_OUTPUT);
    int32_t target_position = encoder_read(); // Avoid a startup jump to zero.
    if (target_position < 0) target_position = 0;
    if (target_position > TOTAL_ENCODER_RANGE) target_position = TOTAL_ENCODER_RANGE;
    int8_t joystick_pos = 0;
    uint64_t last_can_time = time_now();
    uint64_t solenoid_off_time = 0;
    bool button_was_pressed = false;
    bool solenoid_active = false;
    CanMsg rx_msg;

    while (1) {
        // CAN is polled; no CAN interrupt necessary. Never block the control loop.
        if (can_rx(&rx_msg) && rx_msg.id == 0 && rx_msg.length >= 3) {
            joystick_pos = decode_joystick(rx_msg.byte[0]);
            last_can_time = time_now();

            // Map input to servo pulse, with saturation before unsigned arithmetic.
            int32_t servo_us = 900 + ((int32_t)rx_msg.byte[1] - 67) * 1200 / 182;
            if (servo_us < 900) servo_us = 900;
            if (servo_us > 2100) servo_us = 2100;
            servo_set_duty_us((uint32_t)servo_us);

            bool pressed = rx_msg.byte[2] < 10;
            if (pressed && !button_was_pressed && !solenoid_active) {
                PIOC->PIO_SODR = PIN_RELAY_MASK;
                solenoid_off_time = time_now() + msecs(SOLENOID_PULSE_MS);
                solenoid_active = true;
            }
            button_was_pressed = pressed;
        }

        uint64_t now = time_now();
        if (solenoid_active && now >= solenoid_off_time) {
            PIOC->PIO_CODR = PIN_RELAY_MASK;
            solenoid_active = false;
        }
        // Lost CAN must not leave a stale commanded velocity active indefinitely.
        if (now - last_can_time >= msecs(CAN_TIMEOUT_MS)) {
            joystick_pos = 0;
            button_was_pressed = false;
        }

        if (time_take_control_tick()) {
            target_position += joystick_increment(joystick_pos);
            if (target_position > TOTAL_ENCODER_RANGE) target_position = TOTAL_ENCODER_RANGE;
            if (target_position < 0) target_position = 0;

            int32_t current_position = encoder_read();
            int32_t output = pid_compute(&pos_pid, target_position,
                                         current_position, CONTROL_PERIOD_MS / 1000.0f);
            motor_set(output);
        }

        if (ir_check_goal()) {
            printf("Goal! Score: %u\r\n", (unsigned)ir_get_score());
        }
    }
}
