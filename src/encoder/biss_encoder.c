#include "biss_encoder.h"

static int32_t biss_get_position(void);
static int32_t biss_get_velocity(void);
static void biss_reset_position(void);

EncoderInterface biss_encoder = {
    .get_position = biss_get_position,
    .get_velocity = biss_get_velocity,
    .reset_position = biss_reset_position
};

void biss_encoder_init(biss_encoder_t *encoder) {
    // Initialize any necessary hardware for the BiSS encoder
}

static int32_t biss_get_position(void) {
    // Get position data from the BiSS encoder
    int32_t position = 0;
    if (biss_read_position(&position)) {
        return position;
    } else {
        // Handle the error case if the position data couldn't be read
        return 0;
    }
}

static int32_t biss_get_velocity(void) {
    // Get velocity data from the BiSS encoder
    int32_t velocity = 0;
    if (biss_read_velocity(&velocity)) {
        return velocity;
    } else {
        // Handle the error case if the velocity data couldn't be read
        return 0;
    }
}

static void biss_reset_position(void) {
    // Reset the position data in the BiSS encoder
    biss_set_position(0);
}
