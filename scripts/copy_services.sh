#!/bin/bash

script_dir="$(dirname "$(readlink -f "$0")")"

cp $script_dir/launch-micro-ros.service /etc/user/system/launch-micro-ros.service
cp $script_dir/setup_can0.service /etc/user/system/setup_can0.service