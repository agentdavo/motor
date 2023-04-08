#include "mcu.h"

void mcu_gpio_init(void) {
    // Initialize the GPIOs for the motor control pins
    // ...
}

void mcu_adc_init(void) {
    // Initialize the ADCs for the motor current and bus voltage measurements
    // ...
}

void mcu_uart_init(void) {
    // Initialize the UART for communication with external devices
    // ...
}

void mcu_timer_init(uint8_t timer_num, uint32_t freq_hz) {
    // Initialize the timer for the control loop
    // ...
}

void mcu_pwm_init(uint8_t timer_num, uint8_t channel_num, uint32_t freq_hz) {
    // Initialize the PWM for the motor control
    // ...
}

void mcu_adc_start_conversion(uint8_t channel_num) {
    // Start the ADC conversion for the specified channel
    // ...
}

uint16_t mcu_adc_read(uint8_t channel_num) {
    // Read and return the value of the ADC conversion for the specified channel
    // ...
}

void mcu_pwm_set_duty_cycle(uint8_t timer_num, uint8_t channel_num, float duty_cycle) {
    // Set the duty cycle of the specified PWM channel on the specified timer
    // ...
}

uint32_t mcu_timer_get_counter_value(uint8_t timer_num) {
    // Get the current counter value of the specified timer
    // ...
}
