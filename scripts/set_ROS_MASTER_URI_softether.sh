#!/usr/bin/env bash
export ROS_MASTER_URI=http://10.0.0.2:11311
export ROS_IP=10.0.0.102

# Show current config
env |egrep ROS_[IM]
