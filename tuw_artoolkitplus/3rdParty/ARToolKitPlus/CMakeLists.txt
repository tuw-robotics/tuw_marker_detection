cmake_minimum_required ( VERSION 2.6 )
PROJECT(ARToolKitPlus)

include_directories(include)
include_directories(src)
add_subdirectory(src)
add_subdirectory(sample)
add_subdirectory(tools)
