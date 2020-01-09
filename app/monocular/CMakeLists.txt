add_executable(monoORB monocular.cc )
target_link_libraries(monoORB LINK_PUBLIC ORB_SLAM2)
add_subdirectory(ROS)