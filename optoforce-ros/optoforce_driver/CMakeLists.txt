cmake_minimum_required(VERSION 2.8.3)
project(optoforce_driver)


find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  std_srvs
  omd
)



catkin_package(
  INCLUDE_DIRS include
  LIBRARIES optoforce_driver
  CATKIN_DEPENDS geometry_msgs roscpp std_srvs omd
)

 
include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
)


add_library(${PROJECT_NAME}  src/optoforce6DSensor.cpp)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})
add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS})


add_executable(mainOptoforce6DSensor src/mainOptoforce6DSensor.cpp)
target_link_libraries( mainOptoforce6DSensor ${PROJECT_NAME} ${catkin_LIBRARIES})

add_executable(prueba src/prueba.cpp)
target_link_libraries( prueba ${PROJECT_NAME} ${catkin_LIBRARIES})
