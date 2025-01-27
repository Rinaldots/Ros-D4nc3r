#include "motor_controle.h"

float kp = 1, ki = 0.5, total_error, proportional_error;
const int freq = 5000;
const int resolution = 8;

MotorController leftWheel(motor_left_a, motor_left_b, 8, pwm_channel_ml_a, pwm_channel_ml_b); // Initialize with actual values
MotorController rightWheel(motor_right_a, motor_right_b, 8, pwm_channel_mr_a, pwm_channel_mr_b); // Initialize with actual values

void motor_setup() {
  leftWheel = MotorController(motor_left_a, motor_left_b, 8, pwm_channel_ml_a, pwm_channel_ml_b); // Replace with actual values
  rightWheel = MotorController(motor_right_a, motor_right_b, 8, pwm_channel_mr_a, pwm_channel_mr_b); // Replace with actual values
  leftWheel.initPID(kp, ki, 0.1); // Replace 0 with actual kd value
  rightWheel.initPID(kp, ki, 0.1); // Replace 0 with actual kd value
}

MotorController::MotorController(int8_t ForwardPin, int8_t BackwardPin, int tickPerRevolution, int pwmChannel1, int pwmChannel2) {
  this->Forward = ForwardPin;
  this->Backward = BackwardPin;
  this->tick = tickPerRevolution;
  this->pwmChannel1 = pwmChannel1;
  this->pwmChannel2 = pwmChannel2;
  this->EncoderCount.data = 0;
  rpmPrev = 0;
  // Setup PWM channels
  ledcSetup(pwmChannel1, freq, resolution); // 2^8 = 256, 0-255
  ledcSetup(pwmChannel2, freq, resolution); // 2^8 = 256, 0-255

  // Attach motor pins to PWM channels
  ledcAttachPin(ForwardPin, pwmChannel1);
  ledcAttachPin(BackwardPin, pwmChannel2);
}

void MotorController::initPID(float proportionalGain, float integralGain, float derivativeGain) {
  kp = proportionalGain;
  ki = integralGain;
  kd = derivativeGain;
}

float MotorController::getRpm_real() {
  CurrentPosition_2 = getEncoderCount();
  CurrentTime_2 = millis();
  float delta2 = ((float)CurrentTime_2 - PreviousTime_2) / 1.0e3;
  float rmp_real = (CurrentPosition_2 - PreviousPosition_2) / (delta2);
  PreviousPosition_2 = CurrentPosition_2;
  PreviousTime_2 = CurrentTime_2;
  return rmp_real;
}

float MotorController::getRpm() {
  CurrentPosition = getEncoderCount();
  CurrentTime = millis();

  float delta1 = ((float)CurrentTime - PreviousTime) / 1.0e3;
  float velocity = (CurrentPosition - PreviousPosition) / delta1;
  float rpm = (velocity / tick) * 60;

  rpmFilt = 0.70 * rpmFilt + 0.15 * rpm + 0.15 * rpmPrev;
  rpmPrev = rpm;
  PreviousPosition = CurrentPosition;
  PreviousTime = CurrentTime;
  return rpmFilt;
}

float MotorController::pid(float setpoint, float feedback) {
  CurrentTimeforError = millis();
 
  float delta2 = ((float)CurrentTimeforError - PreviousTimeForError) / 1.0e3;

  error = setpoint - feedback;
  eintegral = eintegral + (error * delta2);
  ederivative = (error - previousError) / delta2;

  float control_signal = (kp * error) + (ki * eintegral) + (kd * ederivative);
  previousError = error;
  PreviousTimeForError = CurrentTimeforError;
  return control_signal;
}

void MotorController::moveBase(float ActuatingSignal, int threshold) {
  float signal;
  

  if (ActuatingSignal > 0){
    signal = ActuatingSignal + threshold;
    status = 1;
  }else{
    signal = ActuatingSignal - threshold;
    status = -1;
  }
  if (signal > 255) {
    signal = 255;
  }else if (signal < -255){
    signal = -255;
  }

  if (signal < threshold && signal > 0){
    signal = 0;
  }else if(signal > -threshold && signal < 0){
    signal = 0;
  }

  Serial.print("AcSignal: ");
  Serial.println(signal);

  if (signal > 0) {
    ledcWrite(pwmChannel1, signal);
    ledcWrite(pwmChannel2, 0);
  } else {
    ledcWrite(pwmChannel1, 0);
    ledcWrite(pwmChannel2, -signal);
  }
}

void MotorController::stop() {
  ledcWrite(pwmChannel1, 0);
  ledcWrite(pwmChannel2, 0);
}

int32_t MotorController::getEncoderCount() {
  return EncoderCount.data;
}

int8_t MotorController::getDirection() {
  return status;
}

