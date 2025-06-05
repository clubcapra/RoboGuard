/**
 * @file sensor_data.h
 * @brief Global sensor data structure and definitions for RoboGuard system
 * @author Philippe Desbiens & Benoit Malenfant
 * @date June 5, 2025
 * @version 1.0
 */

#ifndef SENSOR_DATA_H
#define SENSOR_DATA_H

#include <stdint.h>

/** @defgroup SensorConfiguration Sensor Configuration Constants
 *  @brief Configuration constants for sensor array sizes
 *  @{
 */
#define N_THERMISTORS 1         /**< Number of thermistor sensors in the system */
#define N_BATTERY_CELLS 12      /**< Number of battery cells to monitor (12S configuration) */
/** @} */

/**
 * @struct sensor_data_t
 * @brief Global sensor data structure containing all system measurements
 * 
 * This structure holds all sensor readings and system status information
 * that is shared between different modules of the RoboGuard system.
 * Data is updated by the interfaces module and consumed by the micro-ROS
 * communication module for telemetry transmission.
 */
typedef struct{
    /** @defgroup BatteryData Battery Monitoring Data
     *  @brief Battery pack and cell monitoring information
     *  @{
     */
    float battery_cell_voltage[N_BATTERY_CELLS];    /**< Individual cell voltages in volts */
    float battery_temp;                             /**< Battery temperature in degrees Celsius */
    float battery_voltage;                          /**< Total battery pack voltage in volts */
    float battery_current;                          /**< Battery current in amperes (+ = charging, - = discharging) */
    float battery_percent;                          /**< Battery charge percentage (0.0 to 1.0) */
    /** @} */
    
    /** @defgroup EnvironmentalData Environmental Monitoring Data
     *  @brief Ambient environmental sensor readings
     *  @{
     */
    float ambiant_temp;                             /**< Ambient temperature in degrees Celsius */
    float humidity;                                 /**< Relative humidity percentage (0-100%) */
    /** @} */
    
    /** @defgroup SafetyData Safety and Emergency Stop Data
     *  @brief Emergency stop and safety system status
     *  @{
     */
    uint8_t estop_pwr_out;                          /**< Emergency stop power output control (0 = off, 1 = on) */
    bool estop_status_boutons;                      /**< Physical emergency stop button status */
    bool estop_status_stm32;                        /**< STM32 emergency stop output status */
    /** @} */
}sensor_data_t;

/**
 * @brief Global sensor data instance
 * 
 * Global instance of the sensor data structure that contains all current
 * sensor readings and system status. This variable is updated by the
 * interfaces module and read by other system components.
 */
extern sensor_data_t sensor_data;

#endif