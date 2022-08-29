cd ~/PX4-Autopilot &&
source Tools/setup_gazebo.bash $(pwd) $(pwd)/build/px4_sitl_default &&
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$(pwd) &&
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$(pwd)/Tools/sitl_gazebo &&
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$(pwd)/Tools/sitl_gazebo/models &&
roslaunch gazebo_ros empty_world.launch pause:=false