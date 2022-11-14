cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

include_directories(${GAZEBO_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})
list(APPEND CMAKE_CXX_FLAGS "${GAZEBO_CXX_FLAGS}")

#set(OpenCV_DIR "/usr/lib/x86_64-linux-gnu")
#find_package(opencv REQUIRED)
find_package(catkin REQUIRED COMPONENTS cv_bridge image_transport)
include_directories ("/usr/include/opencv4/")

add_library(faceplugin SHARED faceplugin.cc)
target_link_libraries(faceplugin ${GAZEBO_LIBRARIES} ${catkin_LIBRARIES} /usr/lib/x86_64-linux-gnu/libopencv_core.so /usr/lib/x86_64-linux-gnu/libopencv_imgcodecs.so)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")

add_executable(testvideo testvideo.cc)
target_link_libraries(testvideo ${GAZEBO_LIBRARIES} ${catkin_LIBRARIES} /usr/lib/x86_64-linux-gnu/libopencv_core.so /usr/lib/x86_64-linux-gnu/libopencv_imgcodecs.so )