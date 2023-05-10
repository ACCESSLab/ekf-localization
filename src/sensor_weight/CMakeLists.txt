cmake_minimum_required(VERSION 3.0)

project(sensor_weight)

find_package(catkin REQUIRED COMPONENTS 
    roscpp
    std_msgs 
    autoware_msgs
    novatel_oem7_msgs
)

catkin_package(
   CATKIN_DEPENDS roscpp std_msgs autoware_msgs novatel_oem7_msgs
)

include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(sensor_weight src/sensor_weight.cpp src/sensor_weight_node.cpp)

target_link_libraries(sensor_weight ${catkin_LIBRARIES})