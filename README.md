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

TO setup can :
add this line to /boot/firmware/config.txt (under section all)

dtoverlay=mcp251xfd,spi0-0,oscillator=20000000,interrupt=25


TO setup ros agent on boot 

sudo cp ./scripts/launch-micro-ros.service /etc/systemd/system/launch-micro-ros.service
sudo systemctl enable launch-micro-ros.service

check the path in the script corresponds to the path of the project

# Setup can on boot
sudo nano /etc/modules-load.d/can.conf

Content :\ 
can\
can_raw\

sudo systemctl start systemd-networkd\
sudo systemctl enable systemd-networkd\

sudo nano /etc/systemd/network/80-can.network

content:\
[Match]
Name=can0
[CAN]
BitRate=400K

# Different build versions
WATCH OUT\
there are 3 build configs, env:light env:with_micro_ros_precompiled and env:with_micro_ros

"light" is meant to be built on any machine to develop/debug IT WILL NOT COMMUNICATE WITH ROS BE AWARE

"with_micro_ros" is meant to be built on a linux machine (due to micro-ros limitations) and is meant as the release version that is uploaded in the robot.

"env:with_micro_ros_precompiled" uses a pre-built version of micro-ros, you can re-generate it using the gen_precompiled script, it is meant to reduce first build time and allow for developpement on windows


# Troubleshooting

If docker fails to start with this error:
```
Jun 22 19:55:57 Roboguard-pi dockerd[4937]: time="2024-06-22T19:55:57.304899616+02:00" level=info msg="Starting up"
Jun 22 19:55:57 Roboguard-pi dockerd[4937]: time="2024-06-22T19:55:57.547434828+02:00" level=info msg="[graphdriver] using prior storage driver: overlay2"
Jun 22 19:55:57 Roboguard-pi dockerd[4937]: time="2024-06-22T19:55:57.574117206+02:00" level=info msg="Loading containers: start."
Jun 22 19:55:57 Roboguard-pi dockerd[4937]: time="2024-06-22T19:55:57.698761511+02:00" level=warning msg="failed to find iptables" error="exec: \"iptables\": executable file not found in $PATH"
Jun 22 19:55:57 Roboguard-pi dockerd[4937]: time="2024-06-22T19:55:57.704862473+02:00" level=info msg="stopping event stream following graceful shutdown" error="<nil>" module=libcontainerd namespace=moby
Jun 22 19:55:57 Roboguard-pi dockerd[4937]: failed to start daemon: Error initializing network controller: error obtaining controller instance: failed to register "bridge" driver: failed to create NAT chain DOCKER: i>
Jun 22 19:55:57 Roboguard-pi systemd[1]: docker.service: Main process exited, code=exited, status=1/FAILURE
```
See this comment https://github.com/docker/for-linux/issues/1437#issuecomment-1293818806 to solve the issue.
