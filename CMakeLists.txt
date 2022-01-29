cmake_minimum_required(VERSION 2.8.3)
project(socket_streamer)

find_package (catkin REQUIRED COMPONENTS
    roscpp
    std_msgs
    cv_bridge
    image_transport
    sensor_msgs
)

find_package(OpenCV)

catkin_package(
   INCLUDE_DIRS include
   LIBRARIES ${PROJECT_NAME}
   CATKIN_DEPENDS roscpp
   DEPENDS OpenCV
   )

include_directories(${catkin_INCLUDE_DIRS})
include_directories(${OpenCV_INCLUDE_DIRS})

include_directories(include)
add_executable(server_node src/server.cpp)
target_link_libraries(server_node ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} )