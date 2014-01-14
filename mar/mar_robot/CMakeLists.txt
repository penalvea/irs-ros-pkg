cmake_minimum_required(VERSION 2.8.3)
project(mar_robot)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  geometry_msgs
  kdl_conversions 
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS
    geometry_msgs
    roscpp    
    kdl_conversions
  DEPENDS
    boost
)
include_directories(${PROJECT_SOURCE_DIR}/include ${Boost_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS})
link_directories(${catkin_LIBRARY_DIRS} ${Boost_LIBRARY_DIRS})

add_library(${PROJECT_NAME} 
src/robot_utils.cpp
)
target_link_libraries(${PROJECT_NAME} mar_core ${catkin_LIBRARIES} ${Boost_LIBRARIES})
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
