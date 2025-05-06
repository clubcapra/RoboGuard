#include <Arduino.h>
#include <IWatchdog.h>
#include "sensor_data.h"
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME680.h>
#include "interfaces.h"


#define WATCHDOG_TIMEOUT 6000000 //6 second timeout

#ifdef USE_MICRO_ROS
#include "roboguard_micro_ros.h"
#endif

#define DEBUG_LED PC13

sensor_data_t sensor_data;

#ifndef USE_MICRO_ROS
HardwareSerial Serial3(USART3);
#endif
HardwareSerial Serial1(USART1);

void setup() {
  //enable watchdog to reset if stalled
  IWatchdog.begin(WATCHDOG_TIMEOUT);

  setup_interfaces();
  pinMode(DEBUG_LED, OUTPUT);
  digitalWrite(DEBUG_LED,1);

  
  #ifdef USE_MICRO_ROS
  setup_micro_ros();
  #else
  Serial1.setRx(PA10);
  Serial1.setTx(PA9);
  Serial1.begin(9600);
  Serial1.println("Initializing...");
  Serial3.setRx(PC11);
  Serial3.setTx(PC10);
  Serial3.begin(115200);
  #endif
  
  
}


void loop() {
  
  update_interfaces();
  
  //digitalWrite(DEBUG_LED,1);
  #ifdef USE_MICRO_ROS
  update_micro_ros();
  #else
  //Place developpement loop here
  #endif
 
  IWatchdog.reload();
}