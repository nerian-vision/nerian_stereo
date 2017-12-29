#!/bin/bash

# An example how to launch the node with explicitely specified
# parameters.

device_address="192.168.10.10"

# First download the current calibration file from the device
./download_calibration.sh $device_address /tmp/nerian_calib.yaml

# Launch the node with the the given parameters
nerian_stereo_node \
    point_cloud_intensity_channel:=true \
    ros_coordinate_system:=true \
    color_code_disparity_map:=none \
    color_code_legend:=true \
    use_tcp:=false \
    remote_host:=0.0.0.0 \
    remote_port:=7681 \
    local_host:=0.0.0.0 \
    local_port:=7681 \
    calibration_file:=/tmp/nerian_calib.yaml \
    frame:=map \
    max_depth:=-1
