#include <Arduino.h>
#include "motor_controle.h"

MOTOR leftWheel(A_IA, A_IB, 1); // Initialize with actual values
MOTOR rightWheel(B_IA, B_IB, 0); // Initialize with actual values




void IRAM_ATTR a_encoder() {
  unsigned long current_time = millis();
  if (current_time - leftWheel.encoder_interrupt_time > debounce_delay) {
    leftWheel.Enc_count++;
    leftWheel.encoder_interrupt_time = current_time;
  }
}
void IRAM_ATTR b_encoder() {
  unsigned long current_time = millis();
  if (current_time - rightWheel.encoder_interrupt_time > debounce_delay) {
    rightWheel.Enc_count++;
    rightWheel.encoder_interrupt_time = current_time;
  }
}

MOTOR::MOTOR(int8_t PIN_A, int8_t PIN_B, bool inverted) {
  this->PIN_A = PIN_A;
  this->PIN_B = PIN_B;
  this->inverted = inverted;
  this->Enc_count = 0;
  this->inverted = inverted;
  
  ledcAttach(PIN_A, 5000, 10); // 2^10 = 1023, 0-255
  ledcAttach(PIN_B, 5000, 10); // 2^10 = 1023, 0-255
}

void MOTOR::pwm(int PWM, bool Direction) {
  if (inverted) {
    if (Direction) {
      ledcWrite(PIN_B, 0);
      ledcWrite(PIN_A, 1023 - PWM);
    } else {
      ledcWrite(PIN_B, 1023 - PWM);
      ledcWrite(PIN_A, 0);
    }
  } else {
    if (Direction) {
      ledcWrite(PIN_A, 0);
      ledcWrite(PIN_B, PWM);
    } else {
      ledcWrite(PIN_A, PWM);
      ledcWrite(PIN_B, 0);
    }
  }
}

void MOTOR::Calcular_Velocidade() {
  current_time_2 = millis();
  CurrentPosition = Enc_count;
  float delta_time = (current_time_2 - previous_time_2);
  if (delta_time >= 500) {
    int delta_position = CurrentPosition - PreviousPosition;
    this->Velocidade_dv = (delta_position / delta_time) * 1000;   
    PreviousPosition = CurrentPosition;
    previous_time_2 = current_time_2;
  }
}

// Inicializa Pinos
void motor_setup() {
  pinMode(EN_A, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(EN_A), a_encoder, FALLING);
  pinMode(EN_B, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(EN_B), b_encoder, FALLING);
  leftWheel.pwm(0, 1);
  rightWheel.pwm(0, 1);
  
}


