#include <Arduino.h>

#include "roboguard_microros.h"

void setup() {
  pinMode(PA_5, OUTPUT);

  setup_micro_ros();
}

void loop() {
  delay(500);
  digitalWrite(PA_5,update_micro_ros());
}