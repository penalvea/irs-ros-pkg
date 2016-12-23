cmake_minimum_required(VERSION 2.8.3)
project(omd)


find_package(catkin REQUIRED)

find_library(omd_lib NAMES "OMD" PATHS "lib")

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${omd_lib}
)
include_directories(include ${catkin_INCLUDE_DIRS})

link_directories(${catkin_LIBRARY_DIRS})

install( DIRECTORY include/
   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

install( FILES ${omd_lib}
   DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})
