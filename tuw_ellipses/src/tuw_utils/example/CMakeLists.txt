
add_executable(contour main_contour.cpp)
target_link_libraries(contour ${OPENCV_LIBRARIES}  v4r_contour)


add_executable(edgelinker main_edgelinker.cpp)
target_link_libraries(edgelinker ${OPENCV_LIBRARIES}  v4r_edgelinker v4r_canny)
