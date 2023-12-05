export ADRESS_IP=$(hostname -I | awk '{print $1}')
export SERIAL_PORT=/dev/serial0

docker run --env ROS_IP=$ADRESS_IP --net=host --device=$SERIAL_PORT -d --restart always microros/micro-ros-agent:humble serial --dev $SERIAL_PORT 
