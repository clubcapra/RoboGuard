#include <Arduino.h>
#include "sensor_data.h"
#include "interfaces.h"

#ifdef USE_MICRO_ROS
#include "roboguard_micro_ros.h"
#endif

sensor_data_t sensor_data;

void setup() {
  setup_interfaces();
  pinMode(PA_5, OUTPUT);
  #ifdef USE_MICRO_ROS
  setup_micro_ros();
  #else
  //Place developpement init here
  #endif
}

void loop() {
  update_interfaces();
  #ifdef USE_MICRO_ROS
  digitalWrite(PA_5,update_micro_ros());
  #else
  //Place developpement loop here
  #endif
}