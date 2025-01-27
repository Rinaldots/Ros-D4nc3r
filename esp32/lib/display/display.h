#ifndef OLED_DISPLAY_H
#define OLED_DISPLAY_H

#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Arduino.h>


void drawRightArrow();
void drawLeftArrow();
void drawForwardArrow();
void drawBackwardArrow();
void drawStopCircle();

void setupDisplay();
void displayLineFollowing(int &lpwm,int &rpwm);
void drawTopic(uint8_t* arraytopic_bit_map);
void custom_draw(int number);
#endif