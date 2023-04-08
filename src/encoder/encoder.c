#include "encoder.h"
#include "mcu_gpio.h"

// Quadrature encoder lookup table
const int8_t QUAD_LOOKUP[] = {0, -1, 1, 0, 1, 0, 0, -1, -1, 0, 0, 1, 0, 1, -1, 0};

void encoder_init(Encoder* encoder, uint8_t channel_a, uint8_t channel_b) {
    // Store pins
    encoder->channel_a = channel_a;
    encoder->channel_b = channel_b;

    // Set up GPIO pins
    mcu_gpio_init(channel_a, GPIO_Mode_IPU);
    mcu_gpio_init(channel_b, GPIO_Mode_IPU);

    // Set initial state
    encoder->position = 0;
    encoder->velocity = 0;
}

void encoder_update(Encoder* encoder) {
    // Read current state of pins
    uint8_t pin_a_state = mcu_gpio_read(encoder->channel_a);
    uint8_t pin_b_state = mcu_gpio_read(encoder->channel_b);

    // Determine current and previous states as an index into the lookup table
    uint8_t current_state = (pin_a_state << 1) | pin_b_state;
    uint8_t prev_state = ((encoder->position >> 1) & 0x03) | (pin_b_state << 2);

    // Calculate position change from lookup table
    int8_t delta_position = QUAD_LOOKUP[(current_state << 2) | prev_state];

    // Update position and velocity
    encoder->velocity = delta_position;
    encoder->position += delta_position;
}
