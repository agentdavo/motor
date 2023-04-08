#ifndef CONTROL_INTERFACE_H
#define CONTROL_INTERFACE_H

#include "MotorData.h"

// Control algorithm function pointer type
typedef void (*ControlFunction)(uint32_t motor_index, MotorData* data);

// Control algorithm structure
typedef struct {
    ControlFunction init; // Initialization function
    ControlFunction run;  // Execution function
    ControlFunction stop; // Stop function
} ControlAlgorithm;

// Initializes the motor control algorithm
void control_init(uint32_t motor_index, const ControlAlgorithm* algorithm);

// Runs the motor control algorithm for one iteration
void control_run(uint32_t motor_index, MotorData* data);

// Stops the motor control algorithm and releases any resources
void control_stop(uint32_t motor_index);

#endif // CONTROL_INTERFACE_H
