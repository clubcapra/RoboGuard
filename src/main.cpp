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
roboguard::pdu::Client pdu_client(PB10, PC12, 0x31);
bool pdu_test_started = false;
uint32_t pdu_last_test_ms = 0;
uint8_t pdu_test_step = 0;
uint32_t pdu_cycle_count = 0;

/**
 * @brief Print an ApiCommandResult in human-readable form.
 */
static void print_pdu_command_result(bool got_status,
                                     const roboguard::pdu::ApiCommandResult &r) {
  if (!got_status) {
    Serial1.println(" -> WAIT_TIMEOUT");
    return;
  }
  Serial1.print(" -> seq=");
  Serial1.print(r.sequence);
  Serial1.print(" rc=");
  Serial1.print(r.status);
  Serial1.print(" cmd=0x");
  if (r.command < 0x10U) Serial1.print('0');
  Serial1.print(r.command, HEX);
  Serial1.print(" arg0=");
  Serial1.print(r.arg0);
  Serial1.print(" arg1=");
  Serial1.println(r.arg1);
}

/**
 * @brief Send one command from the rotating test list and read its result.
 *        Each call advances pdu_test_step, so the bus is exercised across many
 *        different API codepaths over a few seconds.
 */
static void send_and_check_step(uint8_t step) {
  using namespace roboguard::pdu;

  const char *name = "?";
  bool sent = false;

  switch (step) {
    case 0:
      name = "noop";
      sent = pdu_client.noop();
      break;
    case 1:
      name = "setLedDuty(BRAS,25%)";
      sent = pdu_client.setLedDuty(kLedBras, 25);
      break;
    case 2:
      name = "setLedDuty(AVANT,50%)";
      sent = pdu_client.setLedDuty(kLedAvant, 50);
      break;
    case 3:
      name = "setLedDuty(ARRIERE,75%)";
      sent = pdu_client.setLedDuty(kLedArriere, 75);
      break;
    case 4:
      name = "setLedDuty(EXTRA,100%)";
      sent = pdu_client.setLedDuty(kLedExtra, 100);
      break;
    case 5:
      name = "setAllLeds(40%)";
      sent = pdu_client.setAllLeds(40);
      break;
    case 6:
      name = "setLedPattern(SOLID)";
      sent = pdu_client.setLedPattern(kLedPatternSolid);
      break;
    case 7:
      name = "setLedPattern(HEARTBEAT)";
      sent = pdu_client.setLedPattern(kLedPatternHeartbeat);
      break;
    case 8:
      name = "setEstopVtx(false)";
      sent = pdu_client.setEstopVtx(false);
      break;
    case 9:
      name = "clearFaultLog";
      sent = pdu_client.clearFaultLog();
      break;
    default:
      name = "noop(reset)";
      sent = pdu_client.noop();
      break;
  }

  Serial1.print("[T");
  Serial1.print(pdu_cycle_count);
  Serial1.print('.');
  Serial1.print(step);
  Serial1.print("] ");
  Serial1.print(name);
  if (!sent) {
    Serial1.println(" SEND_FAIL");
    return;
  }

  roboguard::pdu::ApiCommandResult result{};
  const bool got_status = pdu_client.waitForCommandResult(result, 250, 5);
  print_pdu_command_result(got_status, result);
}

static constexpr uint8_t kPduTestStepCount = 10;

/**
 * @brief One full PDU test cycle: read identification then advance one step
 *        through the command rotation.  Called every ~1 s from loop().
 */
static void run_pdu_test_cycle() {
  using namespace roboguard::pdu;

  pdu_last_test_ms = millis();

  // 1) Always read /info first so we immediately spot a dead bus.
  ApiInfo info{};
  const bool info_ok = pdu_client.readInfo(info);

  Serial1.print("[INFO] ");
  Serial1.print(info_ok ? "OK" : "FAIL");
  if (info_ok) {
    Serial1.print(" magic=");
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
    Serial1.print(info.fw_patch);
    Serial1.print(" addr=0x");
    Serial1.print(info.i2c_addr, HEX);
    Serial1.print(" rails=");
    Serial1.print(info.rail_count);
  }
  Serial1.println();

  if (!info_ok) {
    Serial1.println("[ABORT] readInfo failed - skipping commands this cycle");
    return;
  }

  // 2) Run the next command in the rotation.
  send_and_check_step(pdu_test_step);

  // 3) Advance and bookkeep.
  pdu_test_step = static_cast<uint8_t>((pdu_test_step + 1U) % kPduTestStepCount);
  if (pdu_test_step == 0U) {
    ++pdu_cycle_count;
    Serial1.println("--- end of test cycle ---");
  }
}
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

  pdu_client.begin(50000);
  pdu_test_started = true;
  
  Serial3.setRx(PC11);
  Serial3.setTx(PC10);
  Serial3.begin(115200);
  pinMode(A12, OUTPUT);
  digitalWrite(A12, 1);
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
  if (pdu_test_started && (millis() - pdu_last_test_ms) >= 1000) {
    run_pdu_test_cycle();
  }
  #endif
 
  // Reset watchdog timer to prevent system reset
  IWatchdog.reload();
}