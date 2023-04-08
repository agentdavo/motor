#ifndef ENCODER_H
#define ENCODER_H

#include <stdint.h>

// Encoder struct
typedef struct Encoder {
    // Pins
    uint8_t channel_a;
    uint8_t channel_b;

    // Current state
    int16_t position;
    int16_t velocity;
} Encoder;

// Function prototypes
void encoder_init(Encoder* encoder, uint8_t channel_a, uint8_t channel_b);
void encoder_update(Encoder* encoder);

#endif // ENCODER_H
