#ifndef MOTOR_CONTROLE_H
#define MOTOR_CONTROLE_H

#include <Arduino.h>
#include <std_msgs/msg/int32.h>
#include <geometry_msgs/msg/twist.h>
// Pin Definitions
#define motor_left_a 33
#define motor_left_b 32
#define motor_right_b 26
#define motor_right_a 25

// PWM Channels
#define pwm_channel_mr_a 0
#define pwm_channel_mr_b 1
#define pwm_channel_ml_a 2
#define pwm_channel_ml_b 3

class MotorController {
public:
  int8_t Forward;
  int8_t Backward;
  int pwmChannel1;
  int pwmChannel2;
  std_msgs__msg__Int32 EncoderCount;
  volatile long CurrentPosition = 0;
  volatile long PreviousPosition = 0;
  volatile long CurrentTime = 0;
  volatile long PreviousTime = 0;
  volatile long CurrentPosition_2 = 0;
  volatile long PreviousPosition_2 = 0;
  volatile long CurrentTime_2 = 0;
  volatile long PreviousTime_2 = 0;
  volatile long CurrentTimeforError = 0;
  volatile long PreviousTimeForError = 0;
  float rpmFilt;
  float eintegral;
  float ederivative;
  float rpmPrev = 0;
  float kp;
  float ki;
  float kd;
  float error;
  float previousError = 0;
  int tick;
  int status = 0;

  MotorController(int8_t ForwardPin, int8_t BackwardPin, int tickPerRevolution, int pwmChannel1, int pwmChannel2);
  void initPID(float proportionalGain, float integralGain, float derivativeGain);
  float getRpm();
  float getRpm_real();
  float pid(float setpoint, float feedback);
  void moveBase(float ActuatingSignal, int threshold);
  void stop();
  int32_t getEncoderCount();
  int8_t getDirection();
};

extern MotorController leftWheel;
extern MotorController rightWheel;
extern geometry_msgs__msg__Twist msg_motor;
void motor_setup();

#endif // MOTOR_CONTROLE_H