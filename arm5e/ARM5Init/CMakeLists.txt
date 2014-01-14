cmake_minimum_required(VERSION 2.8.3)
project(ARM5Init)

find_package(catkin REQUIRED COMPONENTS roscpp sensor_msgs ARM5Controller)

catkin_package(
  CATKIN_DEPENDS
    roscpp
    sensor_msgs
    ARM5Controller
)
include_directories(${PROJECT_SOURCE_DIR}/include ${catkin_INCLUDE_DIRS})


add_executable(ARM5Init src/arm5_init.cpp)
target_link_libraries(ARM5Init ${catkin_LIBRARIES})
