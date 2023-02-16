#!/bin/bash

source /opt/ros/melodic/setup.bash
source ~/dm-vio-ws/devel/setup.bash
export ROS_IP=192.168.1.236
export ROS_MASTER_URI=http://192.168.1.120:11311
chmod +x /home/nvidia/dm-vio-ws/src/dm-vio-ros/scripts/IMU_publisher.py
rosrun dmvio_ros IMU_publisher.py