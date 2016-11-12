cmake_minimum_required(VERSION 2.8.3)
project(tuw_ellipses)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  dynamic_reconfigure
  image_transport
  image_geometry
  geometry_msgs
  roscpp
  rospy
  tf
  nodelet
  marker_msgs
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)
find_package( OpenCV REQUIRED )
link_libraries(${OpenCV_LIBS})

#find_package(cmake_modules REQUIRED)

find_package(Eigen3 REQUIRED)
include_directories(${Eigen_INCLUDE_DIRS})


#add dynamic reconfigure api
generate_dynamic_reconfigure_options(
  cfg/EllipsesDetection.cfg
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
#  INCLUDE_DIRS include
#  LIBRARIES tuw_ellipses
#  CATKIN_DEPENDS cv_bridge dynamic_reconfigure image_geometry image_transport roscpp rospy tf
#  DEPENDS eigen3 OpenCV
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_library(tuw_utils 
   src/tuw_utils/contour.cpp 
   src/tuw_utils/canny.cpp
   src/tuw_utils/camera.cpp
   src/tuw_utils/ellipse_refinement.cpp
)

## Declare a cpp library
add_library(ellipses
   src/${PROJECT_NAME}/ellipses_detection.cpp
   src/${PROJECT_NAME}/ellipses_draw.cpp
   src/${PROJECT_NAME}/ellipses_parameters.cpp
)

add_library(ellipses_nodelet
   src/ellipses_nodelet.cpp
   src/ellipses_nodelet_parameters.cpp
)

## Declare a cpp executable
add_executable(ellipses_node src/ellipses_node.cpp)

## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
# add_dependencies(tuw_ellipses_node tuw_ellipses_generate_messages_cpp)
add_dependencies(ellipses_nodelet ${PROJECT_NAME}_gencfg) 
# add_dependencies(ellipses_nodelet ${PROJECT_NAME}_gencpp)

## Specify libraries to link a library or executable target against
target_link_libraries(ellipses_nodelet
   ellipses tuw_utils ${catkin_LIBRARIES}
)
target_link_libraries(ellipses_node
   ellipses_nodelet ${catkin_LIBRARIES}
)
#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
# install(TARGETS tuw_aruco tuw_aruco_node
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark cpp header files for installation
#install(DIRECTORY include/${PROJECT_NAME}/
#  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#  FILES_MATCHING PATTERN "*.h"
#  PATTERN ".svn" EXCLUDE
#)

## Mark other files for installation (e.g. launch and bag files, etc.)
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)
install(DIRECTORY bags/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/bags
)