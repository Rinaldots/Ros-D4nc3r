#include "encoder.h"
#include "motor_controle.h"
#include <micro_ros_platformio.h>
const int left_encoder = 14;
const int right_encoder = 27;

// Defina as variáveis globais
volatile int cnt_left = 0;
volatile int cnt_right = 0;

const int encoder_minimum = -2147483000;
const int encoder_maximum = 2147483000;

// Variáveis de debounce
volatile unsigned long last_left_interrupt_time = 0;
volatile unsigned long last_right_interrupt_time = 0;
const unsigned long debounce_delay = 75; // 50 ms de debounce

// Função de interrupçãoencoder_minimum
void IRAM_ATTR leftIntr() {
    unsigned long current_time = millis();
    if (current_time - last_left_interrupt_time > debounce_delay) {
        if(leftWheel.status < 0){
            cnt_left--;
        }else{
            cnt_left++;
        }
        
        //Serial.print("esquerdo: ");
        //Serial.println(cnt_left);
        last_left_interrupt_time = current_time;
        leftWheel.EncoderCount.data = cnt_left;
    }
}

void IRAM_ATTR rightIntr() {
    unsigned long current_time = millis();
    if (current_time - last_right_interrupt_time > debounce_delay) {
        if(rightWheel.status < 0){
            cnt_right--;
        }else{
            cnt_right++;
        }
        //Serial.print("direito: ");
        //Serial.println(rightWheel.status);
        //Serial.println(cnt_right);
        last_right_interrupt_time = current_time;
        rightWheel.EncoderCount.data = cnt_right;
    }
}

void encoder_setup() {
    pinMode(left_encoder, INPUT_PULLUP);
    pinMode(right_encoder, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(left_encoder), leftIntr, FALLING);
    attachInterrupt(digitalPinToInterrupt(right_encoder), rightIntr, FALLING);
}