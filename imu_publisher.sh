source /opt/ros/melodic/setup.bash
source ~/dm_vio_ws/devel/setup.bash
export ROS_IP=192.168.1.236
export ROS_MASTER_URI=http://192.168.1.120:11311
rosrun dm_vio_ws imu_publisher.py