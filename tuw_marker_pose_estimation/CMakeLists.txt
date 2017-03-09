cmake_minimum_required(VERSION 2.8.3)
project(tuw_marker_pose_estimation)

set(CMAKE_CXX_FLAGS "-std=c++0x ${CMAKE_CXX_FLAGS}")

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
    roscpp
    rospy
    std_msgs
    image_geometry
    tf
    cv_bridge
    marker_msgs
    dynamic_reconfigure
)

## System dependencies
find_package( OpenCV REQUIRED )

## Generate dynamic reconfigure parameters
generate_dynamic_reconfigure_options(
        cfg/MarkerPoseEstimation.cfg
        cfg/MarkerMapPoseEstimation.cfg
)

## The catkin_package macro generates cmake config files for your package
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES
#  CATKIN_DEPENDS cv_bridge image_geometry image_transport image_view opencv tf
#  DEPENDS system_lib
)

###########
## Build ##
###########

include_directories(
    include/
    ${catkin_INCLUDE_DIRS}
)

add_executable(pose_estimation_node
    src/pose_estimation_node.cpp
    src/pose_estimation_base.cpp
    src/pose_estimation_parameters.cpp
    src/marker_fiducials.cpp
    src/marker_pose.cpp
)

add_executable(pose_estimation_markermap_node
    src/markermap/pose_estimation_markermap_node.cpp
    src/markermap/pose_estimation_markermap_base.cpp
    src/markermap/pose_estimation_markermap_parameters.cpp
    src/markermap/marker_map_estimator.cpp
    src/markermap/marker_map_config.cpp
    src/marker_fiducials.cpp
    src/marker_pose.cpp
)

add_executable(pose_estimation_combomarker_node
    src/combomarker/pose_estimation_combomarker_node.cpp
    src/marker_fiducials.cpp
    src/marker_pose.cpp
    src/pose_estimation_base.cpp
    src/pose_estimation_parameters.cpp

    src/combomarker/combo_marker_estimator.cpp
)

## Add cmake target dependencies of the executable
add_dependencies(pose_estimation_node ${PROJECT_NAME}_gencfg)
add_dependencies(pose_estimation_markermap_node ${PROJECT_NAME}_gencfg)
add_dependencies(pose_estimation_combomarker_node ${PROJECT_NAME}_gencfg)

## Specify libraries to link a library or executable target against
target_link_libraries(pose_estimation_node
    ${catkin_LIBRARIES}
    ${OpenCV_LIBRARIES}
)
target_link_libraries(pose_estimation_markermap_node
    ${catkin_LIBRARIES}
    ${OpenCV_LIBRARIES}
)
target_link_libraries(pose_estimation_combomarker_node
    ${catkin_LIBRARIES}
    ${OpenCV_LIBRARIES}
)
