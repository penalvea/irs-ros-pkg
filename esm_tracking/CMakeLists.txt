cmake_minimum_required(VERSION 2.8.3)
project(esm_tracking)

## Detect architecture
## http://www.cmake.org/pipermail/cmake/2011-June/045086.html
set(bitness 32)
if(CMAKE_SIZEOF_VOID_P EQUAL 8)
  set(bitness 64)
endif()

find_package(catkin REQUIRED)
### Find external pre-built libraries
#foreach(lib gflags gtest json linear)
  #find_library(${lib}_lib NAMES ${lib} PATHS libs/${bitness})
#endforeach(lib)
#catkin_package(
  #INCLUDE_DIRS include src
  #LIBRARIES rl_pursuit ${gflags_lib} ${gtest_lib} ${json_lib} ${linear_lib}
#)
find_library(esm_lib NAMES "ESMlib" PATHS "lib${bitness}")

catkin_package(
  INCLUDE_DIRS inc${bitness}
  LIBRARIES ${esm_lib}
)
include_directories(inc${bitness})

install( DIRECTORY inc${bitness}/
   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

install( FILES ${esm_lib}
   DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})
