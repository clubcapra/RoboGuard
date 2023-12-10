#include <Arduino.h>
#include <micro_ros_platformio.h>

#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <std_msgs/msg/float32_multi_array.h>
#include <std_msgs/msg/float32.h>
#include <std_srvs/srv/set_bool.h>
#include <sensor_msgs/msg/battery_state.h>


#include "roboguard_micro_ros.h"
#include "sensor_data.h"

#define BATTERY_CAPACITY 6.5 //in Ah

rcl_publisher_t thermistor_pub, battery_pub, ambiant_temp_pub, humidity_pub;
std_msgs__msg__Float32MultiArray thermistor_msg;
sensor_msgs__msg__BatteryState battery_msg;

rcl_service_t estop_service;
std_srvs__srv__SetBool_Response estop_res;
std_srvs__srv__SetBool_Request estop_req;

std_msgs__msg__Float32 ambiant_temp_msg;
std_msgs__msg__Float32 humidity_msg;


rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_timer_t pub_timer;

const int estop_pin = PA12;

int count = 0;

#define RCSOFTCHECK(fn) (fn != RCL_RET_OK)

void estop_callback(const void * request_msg, void * response_msg){
    // Cast messages to expected types
    std_srvs__srv__SetBool_Request * req_in =
        (std_srvs__srv__SetBool_Request *) request_msg;
    std_srvs__srv__SetBool_Response * res_in =
        (std_srvs__srv__SetBool_Response *) response_msg;
    sensor_data.estop_pwr_out = req_in->data;
    //if asked to turn off power, DO IT NOW
    if(!req_in->data){
        digitalWrite(estop_pin, req_in->data);
    }

    res_in->success = true;
}

void timer_callback(rcl_timer_t * timer, int64_t last_call_time) {
    RCLC_UNUSED(last_call_time);
    if (timer != NULL) {
        RCSOFTCHECK(rcl_publish(&thermistor_pub, &thermistor_msg, NULL));
        RCSOFTCHECK(rcl_publish(&battery_pub, &battery_msg, NULL));
        RCSOFTCHECK(rcl_publish(&ambiant_temp_pub, &ambiant_temp_msg, NULL));
        RCSOFTCHECK(rcl_publish(&humidity_pub, &humidity_msg, NULL));
    }
}

int alive = 0;

HardwareSerial Serial3(USART3);

int setup_micro_ros(){
    int error = 0;

    Serial3.setRx(PC11);
    Serial3.setTx(PC10);
    Serial3.begin(115200);
    set_microros_serial_transports(Serial3);

    //initialize messages
    thermistor_msg.data.data = sensor_data.thermistors;
    thermistor_msg.data.size = N_THERMISTORS;
    thermistor_msg.data.capacity = 1;

    battery_msg.capacity = nan("1");
    battery_msg.design_capacity = BATTERY_CAPACITY;
    battery_msg.charge = nan("1");
    battery_msg.power_supply_technology = sensor_msgs__msg__BatteryState__POWER_SUPPLY_TECHNOLOGY_LIPO;
    battery_msg.present = 1;
    battery_msg.power_supply_health = nan("1");
    thermistor_msg.data.data = sensor_data.battery_cell_voltage;
    battery_msg.cell_voltage.size = N_BATTERY_CELLS;
    battery_msg.cell_voltage.capacity = 1;

    battery_msg.cell_temperature.size=N_BATTERY_CELLS;
    if(battery_msg.cell_temperature.data == NULL){
        battery_msg.cell_temperature.data = (float*)malloc(N_BATTERY_CELLS * sizeof(float));
        for(int i = 0; i<N_BATTERY_CELLS; i++){
            battery_msg.cell_temperature.data[i] = nan("1");
        }
    }

    allocator = rcl_get_default_allocator();

    //create init_options
    error += RCSOFTCHECK(rclc_support_init(&support, 0, NULL, &allocator));

    // create node
    error += RCSOFTCHECK(rclc_node_init_default(&node, "RoboGuard_Node", "RoboGuard", &support));

    // create publisher
    error += RCSOFTCHECK(rclc_publisher_init_default(&thermistor_pub,&node,ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray),"thermistors"));
    error += RCSOFTCHECK(rclc_publisher_init_default(&battery_pub,&node,ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, BatteryState),"battery"));
    error += RCSOFTCHECK(rclc_publisher_init_default(&humidity_pub,&node,ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),"humidity"));
    error += RCSOFTCHECK(rclc_publisher_init_default(&ambiant_temp_pub,&node,ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),"ambiant_temp"));

    // create timer,
    const unsigned int timer_timeout = 66;
    error += RCSOFTCHECK(rclc_timer_init_default(&pub_timer,&support,RCL_MS_TO_NS(timer_timeout),timer_callback));

    // create executor
    error += RCSOFTCHECK(rclc_executor_init(&executor, &support.context, 2, &allocator));//ONLY ALLOWS 2 THINGS TO BE ADDED TO EXECUTOR
    error += RCSOFTCHECK(rclc_executor_add_timer(&executor, &pub_timer));

    //Setup estop service
    error += RCSOFTCHECK(rclc_service_init_default(&estop_service, &node, ROSIDL_GET_SRV_TYPE_SUPPORT(std_srvs, srv, SetBool), "set_estop"));
    error += RCSOFTCHECK(rclc_executor_add_service(&executor, &estop_service, &estop_req, &estop_res, estop_callback));

    alive = !error;

    return(alive);
}

int clean_micro_ros(){
    int error = 0;
    rmw_context_t* rmw_context = rcl_context_get_rmw_context(&support.context);
    (void)rmw_uros_set_context_entity_destroy_session_timeout(rmw_context, 0);

    error += RCSOFTCHECK(rcl_timer_fini(&pub_timer));
    error += RCSOFTCHECK(rclc_executor_fini(&executor));
    error += RCSOFTCHECK(rcl_node_fini(&node));
    error += RCSOFTCHECK(rclc_support_fini(&support));

    error += RCSOFTCHECK(rcl_publisher_fini(&thermistor_pub, &node));

    return(!error);
}

int update_micro_ros(){
    //publishing is done in the timer interrupt
    thermistor_msg.data.data = sensor_data.thermistors;
    battery_msg.voltage = sensor_data.battery_voltage;
    battery_msg.current = sensor_data.battery_current;
    battery_msg.temperature = sensor_data.battery_temp;
    battery_msg.cell_voltage.data = sensor_data.battery_cell_voltage;
    battery_msg.percentage = sensor_data.battery_percent;

    humidity_msg.data = sensor_data.humidity;
    ambiant_temp_msg.data = sensor_data.ambiant_temp;

    rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
    if(!alive){
        alive = setup_micro_ros();
    }
    else if(rmw_uros_ping_agent(100, 3) != RMW_RET_OK){
        alive = 0;
        clean_micro_ros();
    }
    return(alive);
}