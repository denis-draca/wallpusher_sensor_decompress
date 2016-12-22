cmake_minimum_required(VERSION 2.8.3)
project(wallpusher_sensor_decompress)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  image_transport
  cv_bridge
  sensor_msgs
)

find_package(OpenCV REQUIRED)

catkin_package(
)

include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(scan 
  src/main.cpp
  src/decompresser.cpp

  include/wallpusher_sensor_decompress/decompresser.h

)

target_link_libraries(scan ${catkin_LIBRARIES} ${OpenCV_LIBS})
add_definitions(-std=c++11)

