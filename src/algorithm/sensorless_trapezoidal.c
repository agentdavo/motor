#include "control_interface.h"
#include "MotorData.h"
#include "mcu.h"


// This example control algorithm implements a sensorless trapezoidal control 
// scheme, which estimates the rotor position using the back-EMF of the motor, 
// and generates trapezoidal commutation signals to control the motor. 
// The algorithm uses a PID controller to adjust the motor current based on the 
// speed error, and limits the current to a maximum value to protect the motor.
// The algorithm also measures the bus voltage and updates the motor data 
// structure with the current speed, target speed, current, and bus voltage.


// Motor parameters
#define POLE_PAIRS 2 // Number of pole pairs
#define MAX_SPEED 1000 // Maximum motor speed in RPM
#define MAX_CURRENT 10 // Maximum motor current in A
#define KP 0.1 // Proportional gain
#define KI 0.01 // Integral gain
#define KD 0.01 // Derivative gain

// Algorithm variables
static float current_speed = 0.0; // Current motor speed in RPM
static float target_speed = 0.0; // Target motor speed in RPM
static float target_current = 0.0; // Target motor current in A
static float error = 0.0; // Speed error
static float integral = 0.0; // Integral term for PID control
static float derivative = 0.0; // Derivative term for PID control

// Commutation table for trapezoidal control
static uint8_t commutation_table[6][3] = {
    {1, 0, 0},
    {1, 0, 1},
    {0, 0, 1},
    {0, 1, 1},
    {0, 1, 0},
    {1, 1, 0}
};

// Trapezoidal commutation sequence
static uint8_t commutation_index = 0;

// Trapezoidal commutation function
static void commutate(uint8_t a, uint8_t b, uint8_t c) {
    // Set the motor phase outputs according to the commutation table
    mcu_digital_write(MOTOR_A, a);
    mcu_digital_write(MOTOR_B, b);
    mcu_digital_write(MOTOR_C, c);
}

// Initialization function for sensorless trapezoidal control
static void sensorless_trapezoidal_init(uint32_t motor_index, MotorData* data) {
    // Reset the algorithm variables
    current_speed = 0.0;
    target_speed = 0.0;
    target_current = 0.0;
    error = 0.0;
    integral = 0.0;
    derivative = 0.0;
    commutation_index = 0;

    // Set up the motor phase outputs for trapezoidal commutation
    mcu_pwm_write(MOTOR_PWM_A, 0);
    mcu_pwm_write(MOTOR_PWM_B, 0);
    mcu_pwm_write(MOTOR_PWM_C, 0);
    commutate(commutation_table[commutation_index][0], commutation_table[commutation_index][1], commutation_table[commutation_index][2]);
}

// Execution function for sensorless trapezoidal control
static void sensorless_trapezoidal_run(uint32_t motor_index, MotorData* data) {
    // Measure the motor phase voltages and currents
    float phase_voltage_a = mcu_adc_read(MOTOR_PHASE_A);
    float phase_voltage_b = mcu_adc_read(MOTOR_PHASE_B);
    float phase_voltage_c = mcu_adc_read(MOTOR_PHASE_C);
    float phase_current_a = mcu_adc_read(MOTOR_CURRENT_A);
    float phase_current_b = mcu_adc_read(MOTOR_CURRENT_B);
    float phase_current_c = mcu_adc_read(MOTOR_CURRENT_C);
    
// Estimate the motor speed from the back-EMF of the motor
float back_emf = (phase_voltage_a + phase_voltage_b + phase_voltage_c) / 3.0;
current_speed = (back_emf / (POLE_PAIRS * 2 * PI)) * 60.0;

// Calculate the speed error
error = target_speed - current_speed;

// Calculate the PID control output
integral += error;
derivative = error - derivative;
float pid_output = KP * error + KI * integral + KD * derivative;

// Limit the PID control output to the maximum motor current
if (pid_output > MAX_CURRENT) {
    pid_output = MAX_CURRENT;
} else if (pid_output < -MAX_CURRENT) {
    pid_output = -MAX_CURRENT;
}

// Set the target current for the motor
target_current = pid_output;

// Update the commutation index
commutation_index++;
if (commutation_index >= 6) {
    commutation_index = 0;
}

// Set the motor phase outputs for the next commutation step
commutate(commutation_table[commutation_index][0], commutation_table[commutation_index][1], commutation_table[commutation_index][2]);

// Measure the bus voltage
float bus_voltage = mcu_adc_read(MOTOR_BUS);

// Update the motor data structure
data->speed = current_speed;
data->target_speed = target_speed;
data->current = target_current;
data->bus_voltage = bus_voltage;
}

// Stop function for sensorless trapezoidal control
static void sensorless_trapezoidal_stop(uint32_t motor_index) {
// Set the motor phase outputs to low
mcu_digital_write(MOTOR_A, 0);
mcu_digital_write(MOTOR_B, 0);
mcu_digital_write(MOTOR_C, 0);
}

// Control algorithm structure for sensorless trapezoidal control
static const ControlAlgorithm sensorless_trapezoidal_control = {
.init = sensorless_trapezoidal_init,
.run = sensorless_trapezoidal_run,
.stop = sensorless_trapezoidal_stop
};