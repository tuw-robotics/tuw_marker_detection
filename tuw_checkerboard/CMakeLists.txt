cmake_minimum_required(VERSION 3.5)
project(tuw_checkerboard)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(std_msgs REQUIRED)

include_directories(include)

add_library(composition_nodes SHARED
            src/board_detector_node.cpp
            src/board_pose_node.cpp)
target_compile_definitions(composition_nodes
  PRIVATE "TUW_CHECKERBOARD_DLL")
ament_target_dependencies(composition_nodes rclcpp rclcpp_components std_msgs)

# This package installs libraries without exporting them.
# Export the library path to ensure that the installed libraries are available.
if(NOT WIN32)
  ament_environment_hooks(
    "${ament_cmake_package_templates_ENVIRONMENT_HOOK_LIBRARY_PATH}")
endif()

add_executable(composition_detector src/standalone_detector.cpp)
target_link_libraries(composition_detector composition_nodes)
ament_target_dependencies(composition_detector
  rclcpp)

add_executable(composition_pose src/standalone_pose.cpp)
target_link_libraries(composition_pose composition_nodes)
ament_target_dependencies(composition_pose
  rclcpp)

add_executable(composition_composed src/composed.cpp)
target_link_libraries(composition_composed composition_nodes)
ament_target_dependencies(composition_composed rclcpp class_loader)

install(TARGETS
  composition_nodes
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

install(TARGETS
  composition_detector
  composition_pose
  composition_composed
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
