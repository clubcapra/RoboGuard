/**
 * @file roboguard_micro_ros.cpp
 * @brief Implementation of micro-ROS communication interface for RoboGuard
 * @author Philippe Desbiens & Benoit Malenfant
 * @date June 5, 2025
 */

#include <Arduino.h>
#include <micro_ros_platformio.h>

#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <std_msgs/msg/float32_multi_array.h>
#include <std_msgs/msg/float32.h>
#include <std_srvs/srv/set_bool.h>
#include <std_msgs/msg/bool.h>
#include <sensor_msgs/msg/battery_state.h>

#include "roboguard_micro_ros.h"
#include "sensor_data.h"

/** @defgroup BatteryConstants Battery Configuration Constants
 *  @brief Constants for battery monitoring and reporting
 *  @{
 */
#define BATTERY_CAPACITY 6.5        /**< Battery capacity in Ah */
/** @} */

/** @defgroup TimingConstants Communication Timing Constants
 *  @brief Timing configuration for ROS communication
 *  @{
 */
#define TIMER_TIMEOUT_MS 66         /**< Publisher timer timeout in milliseconds */
#define EXECUTOR_TIMEOUT_MS 100     /**< Executor spin timeout in milliseconds */
#define PING_TIMEOUT_MS 100         /**< Agent ping timeout in milliseconds */
#define PING_ATTEMPTS 3             /**< Number of ping attempts to agent */
/** @} */

/** @defgroup ErrorHandling Error Handling Macros
 *  @brief Macros for ROS error checking
 *  @{
 */
#define RCSOFTCHECK(fn) (fn != RCL_RET_OK)  /**< Soft check for ROS function return codes */
/** @} */

/** @defgroup ROSPublishers ROS Publisher Instances
 *  @brief Publisher objects for sensor data
 *  @{
 */
rcl_publisher_t battery_pub;            /**< Battery state publisher */
rcl_publisher_t ambiant_temp_pub;       /**< Ambient temperature publisher */
rcl_publisher_t humidity_pub;           /**< Humidity publisher */
rcl_publisher_t estop_bt_pub;           /**< Emergency stop button publisher */
rcl_publisher_t estop_stm32_pub;        /**< STM32 emergency stop status publisher */
/** @} */

/** @defgroup ROSMessages ROS Message Instances
 *  @brief Message objects for data publication
 *  @{
 */
sensor_msgs__msg__BatteryState battery_msg;     /**< Battery state message */
std_msgs__msg__Bool estop_bt_msg;               /**< Emergency stop button message */
std_msgs__msg__Bool estop_stm32_msg;            /**< STM32 emergency stop message */
std_msgs__msg__Float32 ambiant_temp_msg;        /**< Ambient temperature message */
std_msgs__msg__Float32 humidity_msg;            /**< Humidity message */
/** @} */

/** @defgroup ROSServices ROS Service Instances
 *  @brief Service objects for remote control
 *  @{
 */
rcl_service_t estop_service;                    /**< Emergency stop service */
std_srvs__srv__SetBool_Response estop_res;      /**< Emergency stop service response */
std_srvs__srv__SetBool_Request estop_req;       /**< Emergency stop service request */
/** @} */

/** @defgroup ROSCore ROS Core Infrastructure
 *  @brief Core ROS objects for communication
 *  @{
 */
rclc_executor_t executor;           /**< ROS executor for callbacks */
rclc_support_t support;             /**< ROS support object */
rcl_allocator_t allocator;          /**< Memory allocator */
rcl_node_t node;                    /**< ROS node */
rcl_timer_t pub_timer;              /**< Publisher timer */
/** @} */

/** @defgroup GlobalVariables Global State Variables
 *  @brief Global variables for system state tracking
 *  @{
 */
const int estop_pin = PA12;         /**< Emergency stop output pin */
int alive = 0;                      /**< ROS connection status flag */
HardwareSerial Serial3(USART3);    /**< Serial interface for micro-ROS */
/** @} */

/**
 * @brief Emergency stop service callback
 * 
 * Handles incoming emergency stop service requests from ROS.
 * Immediately applies emergency stop if power-off is requested.
 */
void estop_callback(const void * request_msg, void * response_msg){
    std_srvs__srv__SetBool_Request * req_in = (std_srvs__srv__SetBool_Request *) request_msg;
    std_srvs__srv__SetBool_Response * res_in = (std_srvs__srv__SetBool_Response *) response_msg;
    
    sensor_data.estop_pwr_out = req_in->data;
    
    // If asked to turn off power, DO IT NOW
    if(!req_in->data){
        digitalWrite(estop_pin, req_in->data);
    }

    res_in->success = true;
}

/**
 * @brief Timer callback for periodic data publishing
 * 
 * Publishes all sensor data at regular intervals defined by timer configuration.
 */
void timer_callback(rcl_timer_t * timer, int64_t last_call_time) {
    RCLC_UNUSED(last_call_time);
    if (timer != NULL) {
        RCSOFTCHECK(rcl_publish(&battery_pub, &battery_msg, NULL));
        RCSOFTCHECK(rcl_publish(&estop_bt_pub, &estop_bt_msg, NULL));
        RCSOFTCHECK(rcl_publish(&estop_stm32_pub, &estop_stm32_msg, NULL));
        RCSOFTCHECK(rcl_publish(&ambiant_temp_pub, &ambiant_temp_msg, NULL));
        RCSOFTCHECK(rcl_publish(&humidity_pub, &humidity_msg, NULL));
    }
}

int setup_micro_ros(){
    int error = 0;

    // Configure serial transport
    Serial3.setRx(PC11);
    Serial3.setTx(PC10);
    Serial3.begin(115200);
    set_microros_serial_transports(Serial3);

    // Initialize battery message structure
    battery_msg.capacity = sensor_data.battery_capacity;
    battery_msg.design_capacity = BATTERY_CAPACITY;
    battery_msg.charge = nan("1");
    battery_msg.power_supply_technology = sensor_msgs__msg__BatteryState__POWER_SUPPLY_TECHNOLOGY_LIPO;
    battery_msg.present = 1;
    battery_msg.power_supply_health = 0;
    battery_msg.cell_voltage.data = sensor_data.battery_cell_voltage;
    battery_msg.cell_voltage.size = N_BATTERY_CELLS;
    battery_msg.cell_voltage.capacity = 1;
    battery_msg.cell_temperature.data = sensor_data.battery_temp;
    battery_msg.cell_temperature.size = N_THERMISTORS;
    battery_msg.cell_temperature.capacity = 1;
    battery_msg.percentage= nan("1");

    // Check agent connectivity
    if(rmw_uros_ping_agent(PING_TIMEOUT_MS, PING_ATTEMPTS) != RMW_RET_OK){
        return(0);
    }
    
    allocator = rcl_get_default_allocator();

    // Create init options with domain ID
    rcl_init_options_t init_options = rcl_get_zero_initialized_init_options();
    error += RCSOFTCHECK(rcl_init_options_init(&init_options, allocator));
    error += RCSOFTCHECK(rcl_init_options_set_domain_id(&init_options, ROS_DOMAIN_ID));

    // Initialize rclc support object with custom options
    error += RCSOFTCHECK(rclc_support_init_with_options(&support, 0, NULL, &init_options, &allocator));

    // Create node
    error += RCSOFTCHECK(rclc_node_init_default(&node, "RoboGuard_Node", "RoboGuard", &support));

    // Create publishers
    error += RCSOFTCHECK(rclc_publisher_init_default(&battery_pub,&node,ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, BatteryState),"battery"));
    error += RCSOFTCHECK(rclc_publisher_init_default(&estop_bt_pub,&node,ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool),"estop_bt"));
    error += RCSOFTCHECK(rclc_publisher_init_default(&estop_stm32_pub,&node,ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool),"estop_stm32"));
    error += RCSOFTCHECK(rclc_publisher_init_default(&humidity_pub,&node,ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),"humidity"));
    error += RCSOFTCHECK(rclc_publisher_init_default(&ambiant_temp_pub,&node,ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),"ambiant_temp"));

    // Create timer
    error += RCSOFTCHECK(rclc_timer_init_default(&pub_timer,&support,RCL_MS_TO_NS(TIMER_TIMEOUT_MS),timer_callback));

    // Create executor (allows 2 entities: timer + service)
    error += RCSOFTCHECK(rclc_executor_init(&executor, &support.context, 2, &allocator));
    error += RCSOFTCHECK(rclc_executor_add_timer(&executor, &pub_timer));

    // Setup emergency stop service
    error += RCSOFTCHECK(rclc_service_init_default(&estop_service, &node, ROSIDL_GET_SRV_TYPE_SUPPORT(std_srvs, srv, SetBool), "set_estop"));
    error += RCSOFTCHECK(rclc_executor_add_service(&executor, &estop_service, &estop_req, &estop_res, estop_callback));

    alive = !error;
    return(alive);
}

int clean_micro_ros(){
    int error = 0;
    
    // Set context destroy timeout to immediate
    rmw_context_t* rmw_context = rcl_context_get_rmw_context(&support.context);
    (void)rmw_uros_set_context_entity_destroy_session_timeout(rmw_context, 0);

    // Clean up ROS entities in reverse order
    error += RCSOFTCHECK(rcl_timer_fini(&pub_timer));
    error += RCSOFTCHECK(rclc_executor_fini(&executor));
    error += RCSOFTCHECK(rcl_node_fini(&node));
    error += RCSOFTCHECK(rclc_support_fini(&support));

    return(!error);
}

int update_micro_ros(){
    // Update message data with current sensor readings
    battery_msg.voltage = sensor_data.battery_voltage;
    battery_msg.current = sensor_data.battery_current;
    battery_msg.temperature = sensor_data.bms_temp;
    battery_msg.cell_voltage.data = sensor_data.battery_cell_voltage;
    battery_msg.cell_temperature.data=sensor_data.battery_temp;

    estop_stm32_msg.data = sensor_data.estop_status_stm32;

    humidity_msg.data = sensor_data.humidity;
    ambiant_temp_msg.data = sensor_data.ambiant_temp;

    // Process callbacks and publish data
    rclc_executor_spin_some(&executor, RCL_MS_TO_NS(EXECUTOR_TIMEOUT_MS));
    
    // Handle connection recovery
    if(!alive){
        alive = setup_micro_ros();
    }
    else if(rmw_uros_ping_agent(PING_TIMEOUT_MS, PING_ATTEMPTS) != RMW_RET_OK){
        alive = 0;
        clean_micro_ros();
    }
    
    return(alive);
}