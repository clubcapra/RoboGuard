systemctl stop --user micro-ros.service
sudo cp /home/capra/RoboGuard/scripts/micro-ros.service /etc/systemd/user/micro-ros.service
sudo chmod +x /etc/systemd/user/micro-ros.service
sudo chown capra:capra /etc/systemd/user/micro-ros.service
systemctl --user daemon-reload
systemctl start --user micro-ros.service
