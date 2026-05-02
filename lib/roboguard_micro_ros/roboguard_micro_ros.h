/**
 * @file roboguard_micro_ros.h
 * @brief Nanopb interface for RoboGuard system communication
 * @author Philippe Desbiens & Benoit Malenfant
 * @date June 5, 2025
 * @version 1.0
 */

#ifndef ROBOGUARD_MICRO_ROS_H
#define ROBOGUARD_MICRO_ROS_H

/** @defgroup ROSConfiguration ROS Configuration Constants
 *  @brief Legacy communication constants kept for compatibility
 *  @{
 */
#define ROS_DOMAIN_ID 96            /**< ROS domain ID for network isolation */
/** @} */

/**
 * @brief Initialize Nanopb communication system
 * 
 * Keeps legacy function name for backward compatibility.
 * Sets up serial transport and internal state used for framed
 * protobuf telemetry and command handling.
 * 
 * @return 1 if successful, 0 if initialization failed
 */
int setup_micro_ros();

/**
 * @brief Clean up Nanopb communication resources
 * 
 * Keeps legacy function name for backward compatibility.
 * Properly closes communication resources.
 * 
 * @return 1 if cleanup successful, 0 if errors occurred
 */
int clean_micro_ros();

/**
 * @brief Update Nanopb communication
 * 
 * Keeps legacy function name for backward compatibility.
 * Updates telemetry payload with current sensor readings and handles
 * incoming emergency stop commands.
 * 
 * @return 1 if communication active, 0 if connection inactive
 */
int update_micro_ros();

#endif