#ifndef ENCODER_H
#define ENCODER_H

#include <Arduino.h>

// Variáveis para contar as interrupções
extern volatile int cnt_left;
extern volatile int cnt_right;
extern unsigned long long time_offset;

// Função de interrupção
void IRAM_ATTR leftIntr();
void IRAM_ATTR rightIntr();

// Função de configuração do encoder
void encoder_setup();

// Função para obter RPM
float getRpm();



#endif // ENCODER_H