systemctl stop --user micro-ros.service
cp /home/capra/RoboGuard/scripts/micro-ros.service /etc/systemd/user/micro-ros.service
systemctl start --user micro-ros.service
