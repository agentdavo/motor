#ifndef MCUDEFS_H
#define MCUDEFS_H

#include <stdint.h>

// Define pin mode and pin state types
typedef enum {
    INPUT,
    OUTPUT,
    INPUT_PULLUP,
    INPUT_PULLDOWN,
} pin_mode_t;

typedef enum {
    LOW,
    HIGH,
} pin_state_t;

// Define PWM channel type
typedef uint32_t pwm_channel_t;

// Define ADC channel type
typedef uint32_t adc_channel_t;

// Define UART channel type
typedef uint32_t uart_channel_t;

#endif // MCUDEFS_H
