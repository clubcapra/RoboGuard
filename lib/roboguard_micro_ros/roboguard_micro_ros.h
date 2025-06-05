/**
 * @file roboguard_micro_ros.h
 * @brief Micro-ROS interface for RoboGuard system communication
 * @author Philippe Desbiens & Benoit Malenfant
 * @date June 5, 2025
 * @version 1.0
 */

#ifndef ROBOGUARD_MICRO_ROS_H
#define ROBOGUARD_MICRO_ROS_H

/** @defgroup ROSConfiguration ROS Configuration Constants
 *  @brief Configuration constants for ROS communication
 *  @{
 */
#define ROS_DOMAIN_ID 96            /**< ROS domain ID for network isolation */
/** @} */

/**
 * @brief Initialize micro-ROS communication system
 * 
 * Sets up serial transport, initializes ROS node, creates publishers,
 * services, and executor for RoboGuard sensor data communication.
 * 
 * @return 1 if successful, 0 if initialization failed
 */
int setup_micro_ros();

/**
 * @brief Clean up micro-ROS resources
 * 
 * Properly destroys all ROS entities, timers, executors, and nodes
 * to free allocated memory and close connections.
 * 
 * @return 1 if cleanup successful, 0 if errors occurred
 */
int clean_micro_ros();

/**
 * @brief Update micro-ROS communication
 * 
 * Updates message data with current sensor readings, handles service
 * callbacks, and manages connection health with ROS agent.
 * 
 * @return 1 if communication active, 0 if connection lost
 */
int update_micro_ros();

#endif