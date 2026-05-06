/**
 * @file main.cpp
 * @brief Main application entry point for RoboGuard system
 * @author Your Name
 * @date June 5, 2025
 * @version 1.0
 * 
 * This file contains the main application logic for the RoboGuard system,
 * including initialization of all subsystems, watchdog configuration,
 * and the main execution loop that handles sensor updates and communication.
 */

#include <Arduino.h>
#include <IWatchdog.h>
#include "sensor_data.h"
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME680.h>
#include "interfaces.h"
#ifndef USE_MICRO_ROS
#include "pdu_i2c_api.h"
#endif

// Note: Using Adafruit_BME680 library which supports both BME680 and BME688

/** @defgroup SystemConfiguration System Configuration Constants
 *  @brief Main system configuration parameters
 *  @{
 */
#define WATCHDOG_TIMEOUT 6000000    /**< Watchdog timeout in microseconds (6 seconds) */
#define DEBUG_LED PC13              /**< Debug LED pin for system status indication */
/** @} */

#ifdef USE_MICRO_ROS
#include "roboguard_micro_ros.h"
#endif

/** @defgroup GlobalVariables Global System Variables
 *  @brief Global variables for system operation
 *  @{
 */
sensor_data_t sensor_data;          /**< Global sensor data structure instance */
/** @} */

/** @defgroup SerialInterfaces Serial Communication Interfaces
 *  @brief Serial port instances for different communication channels
 *  @{
 */
#ifndef USE_MICRO_ROS
HardwareSerial Serial3(USART3);    /**< Serial3 for debug/development communication */
#endif
HardwareSerial Serial1(USART6);    /**< Serial1 for general purpose communication */
/** @} */

#ifndef USE_MICRO_ROS
roboguard::pdu::Client pdu_client(Wire,0x31);
bool pdu_test_started = false;
uint32_t pdu_last_test_ms = 0;
#endif

/**
 * @brief System initialization function
 * 
 * Initializes all system components including:
 * - Hardware watchdog timer for system reliability
 * - Sensor interfaces and GPIO configuration
 * - Debug LED for visual status indication
 * - Communication interfaces (micro-ROS or serial debug)
 * 
 * The function configures different communication modes based on
 * compilation flags (USE_MICRO_ROS).
 */
void setup() {
  // Enable watchdog to reset system if stalled
  IWatchdog.begin(WATCHDOG_TIMEOUT);

  // Initialize all sensor interfaces and GPIO
  setup_interfaces();
  
  // Configure debug LED
  pinMode(DEBUG_LED, OUTPUT);
  digitalWrite(DEBUG_LED, 1);  // Turn on LED to indicate initialization

  #ifdef USE_MICRO_ROS
  // Initialize micro-ROS communication for production
  setup_micro_ros();
  #else
  // Initialize serial interfaces for development/debug
  Serial1.setRx(PC7);
  Serial1.setTx(PC6);
  Serial1.begin(9600);
  Serial1.println("Initializing...");

  Wire.setSCL(PB10);
  Wire.setSDA(PC12);
  pdu_client.begin(100000);
  pdu_test_started = true;
  
  Serial3.setRx(PC11);
  Serial3.setTx(PC10);
  Serial3.begin(115200);
  #endif
}

/**
 * @brief Main system execution loop
 * 
 * Continuously executes the following operations:
 * 1. Updates all sensor readings and system status
 * 2. Handles communication (micro-ROS or development debug)
 * 3. Reloads watchdog timer to prevent system reset
 * 
 * The loop behavior changes based on compilation configuration:
 * - With USE_MICRO_ROS: Runs production ROS communication
 * - Without USE_MICRO_ROS: Provides development/debug placeholder
 */
void loop() {
  // Update all sensor readings and system status
  update_interfaces();
  
  #ifdef USE_MICRO_ROS
  // Handle micro-ROS communication and callbacks
  update_micro_ros();
  #else
  if (pdu_test_started && (millis() - pdu_last_test_ms) >= 2000) {
    pdu_last_test_ms = millis();

    roboguard::pdu::ApiInfo info{};
    roboguard::pdu::ApiCommandResult command_result{};
    bool info_ok = pdu_client.readInfo(info);
    bool noop_ok = pdu_client.noop();
    bool status_ok = pdu_client.waitForCommandResult(command_result, 250, 5);

    Serial1.print("PDU info: ");
    Serial1.println(info_ok ? "OK" : "FAIL");
    if (info_ok) {
      Serial1.print("  magic=");
      Serial1.write(reinterpret_cast<const uint8_t *>(info.magic), 4);
      Serial1.print(" proto=");
      Serial1.print(info.protocol_major);
      Serial1.print('.');
      Serial1.print(info.protocol_minor);
      Serial1.print(" fw=");
      Serial1.print(info.fw_major);
      Serial1.print('.');
      Serial1.print(info.fw_minor);
      Serial1.print('.');
      Serial1.println(info.fw_patch);
    }

    Serial1.print("PDU noop: ");
    Serial1.println(noop_ok ? "sent" : "send_fail");
    Serial1.print("PDU status: ");
    Serial1.println(status_ok ? "OK" : "WAIT_FAIL");
    if (status_ok) {
      Serial1.print("  seq=");
      Serial1.print(command_result.sequence);
      Serial1.print(" busy=");
      Serial1.print(command_result.busy);
      Serial1.print(" status=");
      Serial1.print(command_result.status);
      Serial1.print(" cmd=0x");
      Serial1.println(command_result.command, HEX);
    }
  }
  #endif
 
  // Reset watchdog timer to prevent system reset
  IWatchdog.reload();
}