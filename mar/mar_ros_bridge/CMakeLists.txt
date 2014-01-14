cmake_minimum_required(VERSION 2.8.3)
project(mar_ros_bridge)

find_package(catkin REQUIRED COMPONENTS
  tf
  roscpp
  geometry_msgs
  nav_msgs
  kdl_conversions
)
find_package(VISP REQUIRED)
find_package(Boost REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS
    geometry_msgs
    nav_msgs
    tf
    roscpp
    kdl_conversions
  DEPENDS
    boost
    VISP
)
add_definitions(${VISP_DEFINITIONS})
link_directories(${VISP_LIBRARY_DIRS} ${Boost_LIBRARY_DIRS})
include_directories(include ${catkin_INCLUDE_DIRS} ${VISP_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS})

add_library(${PROJECT_NAME} 
src/ROSInterface.cpp
src/Odometry.cpp
src/Wrench.cpp
src/Transform.cpp
)
add_executable(test_odom_sub test/test_odom_sub.cpp)
target_link_libraries(test_odom_sub ${PROJECT_NAME} ${catkin_LIBRARIES} ${Boost_LIBRARIES})
add_executable(test_odom_pub test/test_odom_pub.cpp)
target_link_libraries(test_odom_pub ${PROJECT_NAME} ${catkin_LIBRARIES} ${Boost_LIBRARIES})

install(
  TARGETS
    ${PROJECT_NAME} 
    test_odom_sub
    test_odom_pub
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}
   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
