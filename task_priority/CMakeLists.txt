cmake_minimum_required(VERSION 2.8.3)
project(task_priority)

find_package(orocos_kdl REQUIRED)
find_package(catkin REQUIRED COMPONENTS
  kdl_conversions
  roscpp
  std_msgs
  auv_msgs
  tf
  tf_conversions
)

add_message_files(
FILES
TaskPriority_msg.msg
Task_msg.msg
MultiTask_msg.msg
Error_msg.msg
)

generate_messages(
    DEPENDENCIES
    std_msgs
    )


catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS kdl_conversions roscpp std_msgs
  DEPENDS orocos_kdl
)

###########
## Build ##
###########


include_directories(
  ${PROJECT_SOURCE_DIR}/include
  ${catkin_INCLUDE_DIRS}
)

link_directories(${catkin_LIBRARY_DIRS})
add_library(${PROJECT_NAME}

  src/task_velocity.cpp
  src/jacobian.cpp
  src/task.cpp
  src/goal.cpp
  src/controller.cpp)
#target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${orocos_kdl_LIBRARIES})
#add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS})

#add_executable(pruebas src/pruebas.cpp)
#target_link_libraries(pruebas
#  ${PROJECT_NAME}
#  ${catkin_LIBRARIES}
#)



add_executable(multi_task_priority src/main.cpp)
target_link_libraries(multi_task_priority
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
)
