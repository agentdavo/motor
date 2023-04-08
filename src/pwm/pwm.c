#include "pwm.h"
#include "mcu.h"

void pwm_init(uint8_t timer_num, uint8_t channel_num, uint32_t freq_hz, uint32_t steps, float* duty_cycles) {
    // Configure the PWM timer
    mcu_timer_init(timer_num, freq_hz);

    // Calculate the PWM period and duty cycle values
    uint32_t period = mcu_timer_get_period(timer_num);
    uint32_t step_size = period / steps;
    uint32_t* duty_values = malloc(steps * sizeof(uint32_t));
    for (int i = 0; i < steps; i++) {
        duty_values[i] = duty_cycles[i] * step_size;
    }

    // Configure the PWM channel for multi-step PFM mode
    mcu_pwm_init(timer_num, channel_num, duty_values, steps);

    // Free the duty cycle array
    free(duty_values);
}

void pwm_set_duty_cycle(uint8_t timer_num, uint8_t channel_num, float duty_cycle) {
    // Set the duty cycle of the specified PWM channel on the specified timer
    mcu_pwm_set_duty_cycle(timer_num, channel_num, duty_cycle);
}
