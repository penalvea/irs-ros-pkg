cmake_minimum_required(VERSION 2.8.3)
project(mar_robot_arm5e)

find_package(orocos_kdl REQUIRED)
find_package(catkin REQUIRED COMPONENTS
  kdl_conversions
  roscpp
  geometry_msgs
  mar_robot
  mar_core
  arm5_controller
  cmake_modules
)
find_package(Eigen REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS
    kdl_conversions
    geometry_msgs
    roscpp
    mar_robot
    mar_core
    arm5_controller
  DEPENDS
    Boost
    orocos_kdl
)

include_directories(${PROJECT_SOURCE_DIR}/include ${Boost_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS})
link_directories(${Boost_LIBRARY_DIRS} ${catkin_LIBRARY_DIRS})

add_library(${PROJECT_NAME} 
src/ARM5Solvers.cpp
src/ARM5Arm.cpp
)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${orocos_kdl_LIBRARIES})
add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS})

install(
  TARGETS
    ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}
   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
   
