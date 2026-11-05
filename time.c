#include <stddef.h>
#include <stdint.h>
#include <stdlib.h>
#include <stdbool.h>
#include <sam.h>

#include "time.h"

// SysTick is only a timebase and periodic scheduler. All motor/PID work is in main.
static uint64_t calib;
static volatile uint64_t now = 0;
static volatile uint8_t control_due = 0;
static uint8_t ms_counter = 0;

void time_init(void) {
    calib = SystemCoreClock / 1000u;  // 1 ms at the configured CPU clock
    SysTick->CTRL = 0;
    SysTick->LOAD = (uint32_t)calib - 1u;
    SysTick->VAL = 0;
    ms_counter = 0;
    control_due = 0;
    NVIC_SetPriority(SysTick_IRQn, 2);
    SysTick->CTRL = SysTick_CTRL_CLKSOURCE_Msk |
                    SysTick_CTRL_TICKINT_Msk | SysTick_CTRL_ENABLE_Msk;
}

void SysTick_Handler(void) {
    now += calib;
    if (++ms_counter >= 50u) {
        ms_counter = 0;
        control_due = 1; // Coalesce delayed ticks: don't burst motor updates.
    }
}

bool time_take_control_tick(void) {
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    bool due = (control_due != 0);
    control_due = 0;
    if (!primask) __enable_irq();
    return due;
}

uint64_t time_now(void) {
	uint32_t primask = __get_PRIMASK();
    __disable_irq();
    uint64_t base = now;
    uint32_t val = SysTick->VAL;
    uint32_t pending = SCB->ICSR & SCB_ICSR_PENDSTSET_Msk;
    if (!primask) __enable_irq();
    // If the millisecond has elapsed but its IRQ hasn't executed yet, account for it.
    if (pending) return base + 2u * calib - val - 1u;
    return base + calib - val - 1u;
}

uint64_t usecs(uint64_t s)   { return s * calib / 1000; }
uint64_t msecs(uint64_t s)   { return s * calib; }
uint64_t seconds(uint64_t s) { return s * 1000 * calib; }
uint64_t minutes(uint64_t s) { return s * 60 * 1000 * calib; }
uint64_t hours(uint64_t s)   { return s * 60 * 60 * 1000 * calib; }

float totalUsecs(uint64_t t)   { return t / (1.0 * calib / 1000); }
float totalMsecs(uint64_t t)   { return t / (1.0 * calib); }
float totalSeconds(uint64_t t) { return t / (1.0 * calib * 1000); }
float totalMinutes(uint64_t t) { return t / (1.0 * calib * 1000 * 60); }
float totalHours(uint64_t t)   { return t / (1.0 * calib * 1000 * 60 * 60); }

Time time_split(uint64_t t) {
	Time r;
	uint64_t d;

	d = t / (calib * 1000 * 60 * 60);
	r.hours = (uint16_t)d;
	t -= d * (calib * 1000 * 60 * 60);
	
	d = t / (calib * 1000 * 60);
	r.minutes = (uint8_t)d;
	t -= d * (calib * 1000 * 60);
	
	d = t / (calib * 1000);
	r.seconds = (uint8_t)d;
	t -= d * (calib * 1000);
	
	d = t / calib;
	r.msecs = (uint16_t)d;
	r.ticks = (uint16_t)(t - d * calib);

	return r;
}

uint64_t time_combine(Time t) {
	return t.ticks + msecs(t.msecs) + seconds(t.seconds) + minutes(t.minutes) + hours(t.hours);
}

void time_spinFor(uint64_t duration) {
	time_spinUntil(time_now() + duration);
}

void time_spinUntil(uint64_t then) {
	while (then > time_now()) {}
}

uint64_t ticksPerMs(void) {
	return calib;
}