
SET(SOURCES
  src/arFileGrabber.cpp
  src/calib_inp.cpp
  src/calib_dist.cpp
  src/main.cpp)

add_executable(CameraCalib ${SOURCES})
target_link_libraries(CameraCalib ARToolKitPlus)

