## https://docs.ros.org/en/rolling/Tutorials/Advanced/Security/Introducing-ros2-security.html
## Generate a keystore
# ros2 security create_keystore demo_keystore

## Generate keys and certificates
# ros2 security create_enclave demo_keystore /talker_listener/talker
# ros2 security create_enclave demo_keystore /talker_listener/listener

## Configure environment variables
export ROS_SECURITY_KEYSTORE=/home/markus/projects/ros2/webcam/ws01/demo_keystore 
export ROS_SECURITY_ENABLE=true
export ROS_SECURITY_STRATEGY=Enforce

## Run the talker/listener demo
# ros2 run demo_nodes_cpp talker --ros-args --enclave /talker_listener/talker
# ros2 run demo_nodes_py listener --ros-args --enclave /talker_listener/listener
