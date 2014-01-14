cmake_minimum_required(VERSION 2.8.3)
project(ARM5JoyControl)

find_package(catkin REQUIRED COMPONENTS roscpp sensor_msgs mar_robot_arm5e)
find_package(VISP REQUIRED)

catkin_package(
  CATKIN_DEPENDS
    roscpp
    sensor_msgs
    mar_robot_arm5e
  DEPENDS
    VISP
)
add_definitions(${VISP_DEFINITIONS})
link_directories(${VISP_LIBRARY_DIRS})
include_directories(include ${catkin_INCLUDE_DIRS} ${VISP_INCLUDE_DIRS})

add_executable(ARM5JoyControl src/arm5_joy_control.cpp)
add_executable(ARM5JoyCartesianControl src/arm5_joy_cartesian_control.cpp)

target_link_libraries(ARM5JoyControl ${catkin_LIBRARIES})
target_link_libraries(ARM5JoyCartesianControl ${catkin_LIBRARIES})

install(
  TARGETS
    ARM5JoyControl
    ARM5JoyCartesianControl
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
