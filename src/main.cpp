#include <Arduino.h>
#include <IWatchdog.h>
#include "sensor_data.h"
#include "interfaces.h"


#define WATCHDOG_TIMEOUT 6000000 //6 second timeout

#ifdef USE_MICRO_ROS
#include "roboguard_micro_ros.h"
#endif

sensor_data_t sensor_data;

void setup() {
  //enable watchdog to reset if stalled
  IWatchdog.begin(WATCHDOG_TIMEOUT);

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
  IWatchdog.reload();
}