cmake_minimum_required(VERSION 2.8.3)
project(arm5_power_control)

find_package(catkin REQUIRED COMPONENTS message_generation roscpp std_msgs)

add_service_files(
  DIRECTORY srv
  FILES
  onOff.srv
)

generate_messages(DEPENDENCIES std_msgs)

catkin_package(
  CATKIN_DEPENDS
    roscpp
    std_msgs
    message_runtime
)

add_executable(powerServer src/powerServer.cpp)
add_executable(powerClient src/powerClient.cpp)

target_link_libraries(powerServer ${catkin_LIBRARIES})
target_link_libraries(powerClient ${catkin_LIBRARIES})

add_dependencies(powerServer ${arm5_power_control_EXPORTED_TARGETS})
add_dependencies(powerClient ${arm5_power_control_EXPORTED_TARGETS})

install(
  TARGETS
    powerServer
    powerClient
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
