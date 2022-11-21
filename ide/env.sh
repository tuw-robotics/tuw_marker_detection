export ROS_DISTRO=humble
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

source /opt/ros/$ROS_DISTRO/setup.bash
echo "** ROS2 $ROS_DISTRO initialized with $RMW_IMPLEMENTATION**"
source ${HOME}/projects/ros2/camera/ws00/install/setup.bash
source ${HOME}/projects/ros2/camera/tuw/install/setup.bash
