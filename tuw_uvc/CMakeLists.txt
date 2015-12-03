cmake_minimum_required(VERSION 2.8.3)
project(tuw_uvc)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp rospy dynamic_reconfigure image_transport camera_info_manager message_generation)

## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS thread signals program_options)



find_package( OpenCV REQUIRED )
link_libraries(${OpenCV_LIBS})

add_definitions(-DUSE_SDL -DLINUX -D_GNU_SOURCE=1 -D_REENTRANT )

## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

## To declare and build messages, services or actions from within this
## package, follow these steps:
## * Let MSG_DEP_SET be the set of packages whose message types you use in
##   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
## * In the file package.xml:
##   * add a build_depend and a run_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependencies might have been
##     pulled in transitively but can be declared for certainty nonetheless:
##     * add a build_depend tag for "message_generation"
##     * add a run_depend tag for "message_runtime"
## * In this file (CMakeLists.txt):
##   * add "message_generation" and every package in MSG_DEP_SET to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * add "message_runtime" and every package in MSG_DEP_SET to
##     catkin_package(CATKIN_DEPENDS ...)
##   * uncomment the add_*_files sections below as needed
##     and list every .msg/.srv/.action file to be processed
##   * uncomment the generate_messages entry below
##   * add every package in MSG_DEP_SET to generate_messages(DEPENDENCIES ...)

## Generate messages in the 'msg' folder
add_message_files(
   FILES
   Sphere.msg
#   Message2.msg
)

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
generate_messages(
   DEPENDENCIES
   std_msgs  # Or other packages containing msgs
 )

#add dynamic reconfigure api
generate_dynamic_reconfigure_options(
  cfg/CameraGeneral.cfg
  cfg/CameraLogitech.cfg
  cfg/CameraLogitechSphere.cfg
)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES tuw_uvc tuw_uvc_ros
  CATKIN_DEPENDS message_runtime
#  DEPENDS system_lib
)

###########
## Build ##
###########
## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(include
  ${catkin_INCLUDE_DIRS}
  /usr/include/SDL
  ./
  )

## Declare a cpp library
add_library(tuw_uvc
  src/luvcview/color.c
  src/luvcview/utils.c
  src/luvcview/v4l2uvc.c
)
target_link_libraries(tuw_uvc m SDL v4l2)

add_library(tuw_uvc_ros
  src/tuw_uvc/uvc.cpp 
  src/tuw_uvc/uvc_ros.cpp
)

target_link_libraries(tuw_uvc_ros m v4l2 tuw_uvc tuw_uvc_ros
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES} 
  ${OpenCV_LIBS}
)

## Declare a cpp executable
add_executable(general_node src/general_node.cpp)
add_executable(logitech_node src/logitech_node.cpp)
add_executable(logitech_sphere_node src/logitech_sphere_node.cpp)

add_executable(update_dynamic_reconfigure src/tuw_uvc/uvc.cpp src/update_dynamic_reconfigure.cpp)

## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
add_dependencies(tuw_uvc_ros ${PROJECT_NAME}_gencfg) 
add_dependencies(tuw_uvc_ros ${PROJECT_NAME}_gencpp)

## Specify libraries to link a library or executable target against
target_link_libraries(general_node tuw_uvc_ros
  ${catkin_LIBRARIES})
add_dependencies(general_node tuw_uvc_ros) 
  
target_link_libraries(logitech_node tuw_uvc_ros
  ${catkin_LIBRARIES})
add_dependencies(logitech_node tuw_uvc_ros) 
  
target_link_libraries(logitech_sphere_node tuw_uvc_ros
  ${catkin_LIBRARIES})
  
add_dependencies(logitech_sphere_node tuw_uvc_ros) 
  
target_link_libraries(update_dynamic_reconfigure m v4l2 tuw_uvc 
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES} 
  ${OpenCV_LIBS})
  