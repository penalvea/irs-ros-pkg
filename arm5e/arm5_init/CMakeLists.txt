cmake_minimum_required(VERSION 2.8.3)
project(arm5_init)

find_package(catkin REQUIRED COMPONENTS roscpp sensor_msgs arm5_controller)

catkin_package(
  CATKIN_DEPENDS
    roscpp
    sensor_msgs
    arm5_controller
)
include_directories(${PROJECT_SOURCE_DIR}/include ${catkin_INCLUDE_DIRS})


add_executable(arm5_init src/arm5_init.cpp)
target_link_libraries(arm5_init ${catkin_LIBRARIES})

install(
  TARGETS
    arm5_init
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
