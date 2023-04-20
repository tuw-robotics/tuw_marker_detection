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

add_library(checkerboard_detector_node SHARED
            src/checkerboard_detector_node.cpp)
target_compile_definitions(checkerboard_detector_node
  PRIVATE "TUW_CHECKERBOARD_DLL")
ament_target_dependencies(checkerboard_detector_node rclcpp rclcpp_components std_msgs)

# This package installs libraries without exporting them.
# Export the library path to ensure that the installed libraries are available.
if(NOT WIN32)
  ament_environment_hooks(
    "${ament_cmake_package_templates_ENVIRONMENT_HOOK_LIBRARY_PATH}")
endif()

add_executable(detector src/standalone.cpp)
target_link_libraries(detector checkerboard_detector_node)
ament_target_dependencies(detector
  rclcpp)

add_executable(detector_composed src/composed.cpp)
target_link_libraries(detector_composed checkerboard_detector_node)
ament_target_dependencies(detector_composed rclcpp class_loader)

install(TARGETS
  checkerboard_detector_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

install(TARGETS
  detector
  detector_composed
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
