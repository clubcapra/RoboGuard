#include <Arduino.h>

#ifdef USE_MICRO_ROS
#include "roboguard_microros.h"
#endif

void setup() {
  pinMode(PA_5, OUTPUT);

  #ifdef USE_MICRO_ROS
  setup_micro_ros();
  #endif
  
}

void loop() {
  delay(500);
  #ifdef USE_MICRO_ROS
  digitalWrite(PA_5,update_micro_ros());
  #endif
}