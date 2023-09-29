#ifndef ROBOGUARD_MICRO_ROS_H
#define ROBOGUARD_MICRO_ROS_H

#include <Arduino.h>
#include <micro_ros_platformio.h>

#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <std_msgs/msg/int32.h>

int setup_micro_ros();
int clean_micro_ros();
int update_micro_ros();

#endif