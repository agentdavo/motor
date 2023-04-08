#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H

#include "MotorData.h"

void motor_init(unsigned int num_motors);
void motor_control(MotorData *motors, unsigned int num_motors);

#endif // MOTOR_CONTROL_H
