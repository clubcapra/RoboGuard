#!/bin/bash
export ADRESS_IP=$(hostname -I | awk '{print $1}')
export SERIAL_PORT=/dev/serial0

docker run --env ROS_IP=$ADRESS_IP --env ROS_DOMAIN_ID=96 --net=host --device=$SERIAL_PORT -d --rm microros/micro-ros-agent:humble serial --dev $SERIAL_PORT 
