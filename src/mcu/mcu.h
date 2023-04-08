#ifndef MCU_H
#define MCU_H

#include "mcudefs.h"
#include "mcus.h"

// Function declarations for direct I/O operations
void mcu_pin_mode(pin_t pin, pin_mode_t mode);
void mcu_pin_write(pin_t pin, pin_state_t state);
pin_state_t mcu_pin_read(pin_t pin);

// Function declarations for PWM outputs
void mcu_pwm_init(pwm_channel_t channel, uint32_t freq);
void mcu_pwm_write(pwm_channel_t channel, float duty_cycle);

// Function declarations for ADC inputs
void mcu_adc_init(adc_channel_t channel);
uint16_t mcu_adc_read(adc_channel_t channel);

// Function declarations for communication interfaces
void mcu_uart_init(uart_channel_t channel, uint32_t baud_rate);
void mcu_uart_send(uart_channel_t channel, uint8_t data);
uint8_t mcu_uart_receive(uart_channel_t channel);

// Additional functions for other peripherals can be added here

#endif // MCU_H
