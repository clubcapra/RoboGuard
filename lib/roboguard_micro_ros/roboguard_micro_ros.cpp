#include "roboguard_micro_ros.h"

rcl_publisher_t publisher;
std_msgs__msg__Int32 msg;

rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_timer_t pub_timer;

#define RCSOFTCHECK(fn) (fn != RCL_RET_OK)

void timer_callback(rcl_timer_t * timer, int64_t last_call_time) {
    RCLC_UNUSED(last_call_time);
    if (timer != NULL) {
        RCSOFTCHECK(rcl_publish(&publisher, &msg, NULL));
        msg.data++;
    }
}

int alive = 0;

int setup_micro_ros(){
    int error = 0;

    Serial.begin(115200);
    set_microros_serial_transports(Serial);

    allocator = rcl_get_default_allocator();

    //create init_options
    error += RCSOFTCHECK(rclc_support_init(&support, 0, NULL, &allocator));

    // create node
    error += RCSOFTCHECK(rclc_node_init_default(&node, "micro_ros_platformio_node", "", &support));

    // create publisher
    error += RCSOFTCHECK(rclc_publisher_init_default(&publisher,&node,ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),"micro_ros_platformio_node_publisher"));

    // create timer,
    const unsigned int timer_timeout = 1000;
    error += RCSOFTCHECK(rclc_timer_init_default(&pub_timer,&support,RCL_MS_TO_NS(timer_timeout),timer_callback));

    // create executor
    error += RCSOFTCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
    error += RCSOFTCHECK(rclc_executor_add_timer(&executor, &pub_timer));

    alive = !error;

    return(alive);
}

int clean_micro_ros(){
    int error = 0;
    rmw_context_t* rmw_context = rcl_context_get_rmw_context(&support.context);
    (void)rmw_uros_set_context_entity_destroy_session_timeout(rmw_context, 0);

    error += RCSOFTCHECK(rcl_publisher_fini(&publisher, &node));
    error += RCSOFTCHECK(rcl_timer_fini(&pub_timer));
    error += RCSOFTCHECK(rclc_executor_fini(&executor));
    error += RCSOFTCHECK(rcl_node_fini(&node));
    error += RCSOFTCHECK(rclc_support_fini(&support));

  return(!error);
}

int update_micro_ros(){
    rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
    if(!alive){
        alive = setup_micro_ros();
    }
    else if(rmw_uros_ping_agent(10, 1) != RMW_RET_OK){
        alive = 0;
        clean_micro_ros();
    }
    return(alive);
}