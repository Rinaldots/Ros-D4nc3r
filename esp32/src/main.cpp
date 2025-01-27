#include <Arduino.h>
#include "ros2.h"


void setup() {
  Serial.begin(115200);
  motor_setup();
  
  
  setupDisplay(); 
  
  encoder_setup();  
  
  mpu6050_setup();

  ros_setup();
}

void loop() {
  ros2_loop();
}

