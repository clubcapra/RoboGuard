#!/bin/bash

script_dir="$(dirname "$(readlink -f "$0")")"

sudo cp $script_dir/launch-micro-ros.service /etc/systemd/user/launch-micro-ros.service
sudo cp $script_dir/setup_can0.service /etc/systemd/system/setup_can0.service
