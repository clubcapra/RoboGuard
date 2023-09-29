# Project setup
To build this project, you'll need vscode with the platformio extension, MAKE SURE TO SELECT THE BUILD TARGET YOU WANT (See Different build versions)

# RPI Setup : disable serial shell
do the following : 
    sudo nano /boot/firmware/cmdline.txt
    ***** in the file remove those characters and save "console=serial0,115200"

# RPI Setup : setup ros_agent
1st step : install docker if not already installed
    use ./scripts/install_docker_linux.sh

2nd step : pull the micro_ros_agent image
    use ./scripts/install_micro_ros_agent.sh

3rd step : run micro_ros_agent
    use ./scripts/launch_micro_ros_agent.sh
    This will allow the pi to communicate over serial with roboguard and communicate the node and the topics over the network


# Different build versions
WATCH OUT
there are 2 build configs, env:light and env:with_micro_ros

"light" is meant to be built on any machine to develop/debug IT WILL NOT COMMUNICATE WITH ROS BE AWARE

"with_micro_ros" is meant to be built on a linux machine (due to micro-ros limitations) and is meant as the release version that is uploaded in the robot.