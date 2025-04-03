sudo systemctl stop micro-ros.service
cp /home/capra/RoboGuard/scripts/micro-ros.service /etc/systemd/system/micro-ros.service
sudo systemctl start micro-ros.service
