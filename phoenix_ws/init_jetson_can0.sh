#!/bin/bash
# Initialize Jetson native CAN interface (can0) at 1 Mbps.
# Use this when CubeMars motors are connected via Jetson onboard CAN hardware.
# For USB-to-CAN adapters on PC, use CubeMars/setup_slcan.sh instead.

set -e

sudo modprobe can
sudo modprobe can_raw
sudo ip link set can0 type can bitrate 1000000
sudo ip link set up can0

echo "[init_jetson_can0] can0 is up at 1 Mbps."
