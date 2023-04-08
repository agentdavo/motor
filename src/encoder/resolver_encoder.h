#ifndef RESOLVER_ENCODER_H_
#define RESOLVER_ENCODER_H_

#include "encoder.h"

typedef struct {
    // resolver-specific data
} resolver_encoder_t;

void resolver_encoder_init(resolver_encoder_t *encoder);
int32_t resolver_get_position(void);
int32_t resolver_get_velocity(void);
void resolver_reset_position(void);

#endif /* RESOLVER_ENCODER_H_ */