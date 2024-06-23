#!/bin/bash

script_dir="$(dirname "$(readlink -f "$0")")"

cp $script_dir/launch-micro-ros.service /etc/systemd/system/launch-micro-ros.service
cp $script_dir/setup_can0.service /etc/systemd/system/setup_can0.service