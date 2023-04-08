#ifndef PWM_H
#define PWM_H

#include <stdint.h>

// Initialize the PWM with multi-step PFM mode
void pwm_init(uint8_t timer_num, uint8_t channel_num, uint32_t freq_hz, uint32_t steps, float* duty_cycles);

// Set the duty cycle of the specified PWM channel on the specified timer
void pwm_set_duty_cycle(uint8_t timer_num, uint8_t channel_num, float duty_cycle);

#endif // PWM_H
