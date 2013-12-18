cmake_minimum_required(VERSION 2.8.3)
project(mar_core)

find_package(catkin REQUIRED COMPONENTS roscpp)
find_package(VISP REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS
    roscpp 
  DEPENDS
    boost
    VISP
)

add_definitions(${VISP_DEFINITIONS})
link_directories(${VISP_LIBRARY_DIRS} ${Boost_LIBRARY_DIRS})
include_directories(include ${catkin_INCLUDE_DIRS} ${VISP_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS})



#common commands for building c++ executables and libraries
add_library(${PROJECT_NAME}  
src/CRobot.cpp
src/CPerception.cpp
src/Arm.cpp
src/CAction.cpp
)
#add_library(${PROJECT_NAME} ${${PROJECT_NAME}_SRCS})
#target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${Boost_LIBRARIES})

install(DIRECTORY include/${PROJECT_NAME}
   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
