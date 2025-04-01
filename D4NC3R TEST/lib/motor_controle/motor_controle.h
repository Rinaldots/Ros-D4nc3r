#ifndef MOTOR_CONTROLE_H
#define MOTOR_CONTROLE_H

#include <Arduino.h>
#include <std_msgs/msg/int32.h>

//motor A
#define A_IA 25
#define A_IB 26
#define B_IA 33
#define B_IB 32
#define EN_A 27
#define EN_B 14

const int freq = 5000;
const int resolution = 8;

// Variáveis de debounce
const unsigned long debounce_delay = 50; // 50 ms de debounce


class MOTOR {
  public: 
    float kp = 0.7, ki = 0.2, total_error, proportional_error;
    int8_t PIN_A, PIN_B;
    bool inverted;
    volatile long CurrentPosition = 0;
    volatile long PreviousPosition = 0;
    volatile long CurrentTime = 0;
    volatile long PreviousTime = -100;
    unsigned long current_time_2 = 0;
    unsigned long previous_time_2 = 0;
    volatile int Enc_count = 0;
    volatile unsigned long encoder_interrupt_time = 0;

    double RPM_target = 0; 
  
    double Velocidade_dv;
    double KP = 0.05; 
    double KI = 0; 
    double KD = 0.01; 

    MOTOR(int8_t PIN_A, int8_t PIN_B, bool inverted); 
    double PWM_value = 0;
    void pwm(int PWM, bool Direction);
    void stop_motor();
    void Calcular_Velocidade();
      
}; 

extern MOTOR leftWheel;
extern MOTOR rightWheel;

void motor_setup();

#endif // MOTOR_CONTROLE_H