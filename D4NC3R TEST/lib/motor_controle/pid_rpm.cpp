#include "pid_rpm.h"
#include "motor_controle.h" // Include the header where Wheel is defined

// PID parameters for left and right motors
float kp_left = 10.0, ki_left = 0.1, kd_left = 0.01;
float kp_right = 10.0, ki_right = 0.1, kd_right = 0.01;

// PID state variables for left motor
float integral_left = 0;
float previous_error_left = 0;

// PID state variables for right motor
float integral_right = 0;
float previous_error_right = 0;

// Custom PID function for left motor
void calculate_pid_left_motor() {
    float error = leftWheel.RPM_target - leftWheel.Velocidade_dv;
    integral_left += error;
    float derivative = error - previous_error_left;
    leftWheel.PWM_value = kp_left * error + ki_left * integral_left + kd_left * derivative+400;

    // Clamp PWM value to valid range
    if (leftWheel.PWM_value > 1023) leftWheel.PWM_value = 1023;
    if (leftWheel.PWM_value < 400) leftWheel.PWM_value = 0;

    previous_error_left = error;
}

// Custom PID function for right motor
void calculate_pid_right_motor() {
    float error = rightWheel.RPM_target - rightWheel.Velocidade_dv;
    integral_right += error;
    float derivative = error - previous_error_right;
    rightWheel.PWM_value = kp_right * error + ki_right * integral_right + kd_right * derivative+400;

    // Clamp PWM value to valid range
    if (rightWheel.PWM_value > 1023) rightWheel.PWM_value = 1023;
    if (rightWheel.PWM_value < 400) rightWheel.PWM_value = 0;

    previous_error_right = error;
}
