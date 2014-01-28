cmake_minimum_required(VERSION 2.8.3)
project(mar_examples)


find_package(VISP REQUIRED)
#find_package(Boost REQUIRED COMPONENTS components); 
find_package(catkin REQUIRED COMPONENTS
  roscpp
  mar_core
  mar_perception
  mar_robot_arm5e
  mar_action
)

catkin_package(
  CATKIN_DEPENDS    
    roscpp
    mar_core
    mar_perception
    mar_robot_arm5e
    mar_action  
  DEPENDS
    VISP
)
add_definitions(${VISP_DEFINITIONS})
link_directories(${VISP_LIBRARY_DIRS} ${catkin_LIBRARY_DIRS})
include_directories(${catkin_INCLUDE_DIRS} ${VISP_INCLUDE_DIRS})


add_executable(setJointPositionMar src/control/setJointPosition.cpp)
target_link_libraries(setJointPositionMar ${catkin_LIBRARIES})
add_executable(setJointVelocityMar src/control/setJointVelocity.cpp)
target_link_libraries(setJointVelocityMar ${catkin_LIBRARIES})
add_executable(initArm src/control/initArm.cpp)
target_link_libraries(initArm ${catkin_LIBRARIES})
add_executable(esm_tracking src/perception/esm_tracking.cpp)
target_link_libraries(esm_tracking ${catkin_LIBRARIES})
add_executable(esm_vs_2d src/perception/esm_vs_2d.cpp)
target_link_libraries(esm_vs_2d ${catkin_LIBRARIES})
