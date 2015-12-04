
add_executable(contour main_contour.cpp)
target_link_libraries(contour ${OPENCV_LIBRARIES}  tuw_contour)


add_executable(edgelinker main_edgelinker.cpp)
target_link_libraries(edgelinker ${OPENCV_LIBRARIES}  tuw_edgelinker tuw_canny)
