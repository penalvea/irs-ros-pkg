cmake_minimum_required(VERSION 2.8.3)
project(ARM5Controller)

find_package(catkin REQUIRED COMPONENTS message_generation roscpp sensor_msgs)
find_package(VISP REQUIRED)

add_service_files(
  DIRECTORY srv
  FILES
  setParams.srv
  setPID.srv
  setZero.srv
)

generate_messages()

catkin_package(
  CATKIN_DEPENDS
    roscpp
    sensor_msgs
    message_runtime
  DEPENDS
    VISP
)

add_definitions(${VISP_DEFINITIONS})
link_directories(${VISP_LIBRARY_DIRS} ${Boost_LIBRARY_DIRS})
include_directories(include ${catkin_INCLUDE_DIRS} ${VISP_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS})



add_executable(ARM5Control src/ARM5Control.cpp src/ARM5Coms.cpp)
target_link_libraries(ARM5Control ${catkin_LIBRARIES} ${VISP_LIBRARIES} ${Boost_LIBRARIES})
add_dependencies(ARM5Control ${ARM5Controller_EXPORTED_TARGETS})
