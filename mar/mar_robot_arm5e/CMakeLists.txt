cmake_minimum_required(VERSION 2.8.3)
project(mar_robot_arm5e)

find_package(Eigen REQUIRED)
find_package(catkin REQUIRED COMPONENTS
  kdl_conversions
  roscpp
  geometry_msgs
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS
    kdl_conversions
    geometry_msgs
    roscpp
  DEPENDS
    boost
)

include_directories(${PROJECT_SOURCE_DIR}/include ${Boost_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS})

link_directories(${Boost_LIBRARY_DIRS})
link_directories(${catkin_LIBRARY_DIRS})

#common commands for building c++ executables and libraries
add_library(${PROJECT_NAME} 
src/ARM5Solvers.cpp
src/ARM5Arm.cpp
)
target_link_libraries(${PROJECT_NAME} mar_robot ${catkin_LIBRARIES} ${Boost_LIBRARIES})
#add_dependencies(${MOVEIT_LIB_NAME} ${catkin_EXPORTED_TARGETS})

install(DIRECTORY include/${PROJECT_NAME}
   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
