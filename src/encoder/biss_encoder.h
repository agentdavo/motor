#ifndef BISS_ENCODER_H
#define BISS_ENCODER_H

#include "encoder.h"

typedef struct {
    // Define any additional data needed for BiSS encoder implementation
} biss_encoder_t;

extern EncoderInterface biss_encoder;

void biss_encoder_init(biss_encoder_t *encoder);

#endif /* BISS_ENCODER_H */
