cmake_minimum_required(VERSION 2.8.3)
project(ARM5PowerControl)

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
#target_link_libraries(ARM5Control ${catkin_LIBRARIES})

add_executable(powerServer src/powerServer.cpp)
add_executable(powerClient src/powerClient.cpp)

target_link_libraries(powerServer ${catkin_LIBRARIES})
target_link_libraries(powerClient ${catkin_LIBRARIES})

add_dependencies(powerServer ${ARM5PowerControl_EXPORTED_TARGETS})
add_dependencies(powerClient ${ARM5PowerControl_EXPORTED_TARGETS})
