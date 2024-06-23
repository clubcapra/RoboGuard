#!/bin/bash

# Bring down the CAN interface if it's already up
sudo ip link set can0 down

# Set the bitrate for the CAN interface
sudo ip link set can0 type can bitrate 250000

# Bring up the CAN interface
sudo ip link set can0 up
