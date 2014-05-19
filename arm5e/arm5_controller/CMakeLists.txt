cmake_minimum_required(VERSION 2.8.3)
project(arm5_controller)

find_package(catkin REQUIRED COMPONENTS message_generation roscpp sensor_msgs)
find_package(VISP REQUIRED)

add_service_files(
  DIRECTORY srv
  FILES
  setParams.srv
  setPID.srv
  setZero.srv
  setOneZero.srv
)

add_message_files(
  FILES
  SendMessage.msg
  ReadMessage.msg
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

add_executable(arm5_control src/ARM5Control.cpp src/ARM5Coms.cpp)
target_link_libraries(arm5_control ${catkin_LIBRARIES} ${VISP_LIBRARIES} ${Boost_LIBRARIES})
add_dependencies(arm5_control ${arm5_controller_EXPORTED_TARGETS})

install(
  TARGETS
    arm5_control
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
