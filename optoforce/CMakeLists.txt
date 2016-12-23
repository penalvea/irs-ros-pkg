cmake_minimum_required(VERSION 2.8.3)
project(optoforce)



find_package(catkin REQUIRED COMPONENTS
  roscpp
  omd
  geometry_msgs
)


catkin_package(
	INCLUDE_DIRS include
 	LIBRARIES {PROJECT_NAME}
    CATKIN_DEPENDS omd roscpp geometry_msgs
)


include_directories(
  ${PROJECT_SOURCE_DIR}/include ${catkin_INCLUDE_DIRS}
)
link_directories(${catkin_LIBRARY_DIRS})


add_library(${PROJECT_NAME}  src/optoforce6DSensor.cpp)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})
add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS})


add_executable(mainOptoforce6DSensor src/mainOptoforce6DSensor.cpp)
target_link_libraries( mainOptoforce6DSensor  optoforce ${catkin_LIBRARIES})
