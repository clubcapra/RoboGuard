# Project setup
To build this project, you'll need vscode with the platformio extension, MAKE SURE TO SELECT THE BUILD TARGET YOU WANT (See Different build versions)

# RPI Setup : disable serial shell
do the following : \
    sudo nano /boot/firmware/cmdline.txt\
    ***** in the file remove those characters and save "console=serial0,115200"

Linux, you might need to install this:
sudo apt install -y python3-venv

add user to dialout
sudo usermod -aG dialout capra

add udev rule for st-link
sudo nano /etc/udev/rules.d/99-st-link-v3.rules
insert content of file from here: 
https://github.com/stlink-org/stlink/blob/master/config/udev/rules.d/49-stlinkv3.rules

# RPI Setup : setup ros_agent
//dockerd-rootless-setuptool.sh install


1st step : install docker if not already installed\
    use ./scripts/install_docker_linux.sh

2nd step : add user to docker group\
    sudo usermod -aG docker capra

3nd step : pull the micro_ros_agent image\
    use ./scripts/install_micro_ros_agent.sh

4th step : run micro_ros_agent\
    use ./scripts/launch_micro_ros_agent.sh\
    This will allow the pi to communicate over serial with roboguard and communicate the node and the topics over the network


# Different build versions
WATCH OUT\
there are 3 build configs, env:light env:with_micro_ros_precompiled and env:with_micro_ros

"light" is meant to be built on any machine to develop/debug IT WILL NOT COMMUNICATE WITH ROS BE AWARE

"with_micro_ros" is meant to be built on a linux machine (due to micro-ros limitations) and is meant as the release version that is uploaded in the robot.

"env:with_micro_ros_precompiled" uses a pre-built version of micro-ros, you can re-generate it using the gen_precompiled script, it is meant to reduce first build time and allow for developpement on windows