add_executable(monoORB_ros src/MonocularROS.cc src/main.cc)
target_link_libraries(monoORB_ros LINK_PUBLIC ORB_SLAM2)
target_include_directories(monoORB_ros PUBLIC include)