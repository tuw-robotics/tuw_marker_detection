cmake_minimum_required(VERSION 2.8.3)
project(tuw_artoolkitplus)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED roscpp rospy image_geometry image_transport tf cv_bridge dynamic_reconfigure message_generation geometry_msgs )

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)
find_package( OpenCV REQUIRED )

# create a tracker that does:
#  - 6x6 sized marker images
#  - samples at a maximum of 6x6
#  - works with luminance (gray) images
#  - can load a maximum of 1 pattern
#  - can detect a maximum of 8 patterns in one image
SET(AR_TRACKER_PARAM "6, 6, 6, 1, 16" CACHE STRING "Tracker parameter maker size x, y, sample size, input format, max load, max pattern" )
add_definitions(-DAR_TRACKER_PARAM=${AR_TRACKER_PARAM})



add_message_files(
  DIRECTORY msg
  FILES
  TransformArrayStamped.msg
)
  
generate_messages(
  DEPENDENCIES 
  geometry_msgs
)

#add dynamic reconfigure api
generate_dynamic_reconfigure_options(
  cfg/ar_parameter.cfg
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
#  LIBRARIES tuw_artoolkitplus
#  CATKIN_DEPENDS other_catkin_pkg
  DEPENDS message_runtime geometry_msgs
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
 include
 ./3rdParty/ARToolKitPlus/include/
  ${catkin_INCLUDE_DIRS}
)

INCLUDE_DIRECTORIES(3rdParty/ARToolKitPlus/include)

SET(SOURCES
  3rdParty/ARToolKitPlus/src/MemoryManager.cpp
  3rdParty/ARToolKitPlus/src/DLL.cpp
  3rdParty/ARToolKitPlus/src/librpp/rpp.cpp
  3rdParty/ARToolKitPlus/src/librpp/rpp_quintic.cpp
  3rdParty/ARToolKitPlus/src/librpp/rpp_vecmat.cpp
  3rdParty/ARToolKitPlus/src/librpp/rpp_svd.cpp
  3rdParty/ARToolKitPlus/src/librpp/librpp.cpp
  3rdParty/ARToolKitPlus/src/extra/Profiler.cpp
  3rdParty/ARToolKitPlus/src/extra/FixedPoint.cpp)
	

add_library(ARToolKitPlus SHARED ${SOURCES})


add_executable(artoolkitplus_node
  src/artoolkitplus_node.cpp
  src/artoolkitplus.cpp
  src/artoolkitplus_draw.cpp
  src/artoolkitplus_parameters.cpp
)

## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
# add_dependencies(artoolkitplus_node tuw_artoolkitplus_generate_messages_cpp)
add_dependencies(artoolkitplus_node ${PROJECT_NAME}_gencfg) 
add_dependencies(artoolkitplus_node ${PROJECT_NAME}_gencpp)

## Specify libraries to link a library or executable target against
target_link_libraries(artoolkitplus_node
  ARToolKitPlus 
  ${catkin_LIBRARIES}
)
