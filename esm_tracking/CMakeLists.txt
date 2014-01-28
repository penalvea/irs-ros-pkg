cmake_minimum_required(VERSION 2.8.3)
project(esm_tracking)

## Detect architecture
## http://www.cmake.org/pipermail/cmake/2011-June/045086.html
set(MY_DOWNLOAD_PATH "${PROJECT_SOURCE_DIR}/library.tar.gz")

set(bitness 32)
set(URL ftp://ftp-sop.inria.fr/evolution/malis/software/ESM/ESMkitLin_0_4.tar.gz)
set(EXTRACTED ESMkitLin)
if(CMAKE_SIZEOF_VOID_P EQUAL 8)
  set(bitness 64)
  set(URL ftp://ftp-sop.inria.fr/evolution/malis/software/ESM/ESMkitLin_x86_64_v0.4.1.tar.gz)
  set(EXTRACTED ESMkitLin_x86_64_v0.4.1)
endif()

IF(NOT EXISTS "${PROJECT_SOURCE_DIR}/downloaded")
  file(DOWNLOAD "${URL}" "${MY_DOWNLOAD_PATH}")

  execute_process(
    COMMAND ${CMAKE_COMMAND} -E tar xzf "${MY_DOWNLOAD_PATH}" 
    WORKING_DIRECTORY "${PROJECT_SOURCE_DIR}")

  execute_process(
    COMMAND ${CMAKE_COMMAND} -E rename "${EXTRACTED}/inc" "inc${bitness}"
    WORKING_DIRECTORY "${PROJECT_SOURCE_DIR}")

  execute_process(
    COMMAND ${CMAKE_COMMAND} -E rename "${EXTRACTED}/lib" "lib${bitness}"
    WORKING_DIRECTORY "${PROJECT_SOURCE_DIR}")

  execute_process(
    COMMAND ${CMAKE_COMMAND} -E rename "lib${bitness}/ESMlib.a" "lib${bitness}/libESMlib.a"
    WORKING_DIRECTORY "${PROJECT_SOURCE_DIR}")

  execute_process(
    COMMAND ${CMAKE_COMMAND} -E rename "${EXTRACTED}/README" "README"
    WORKING_DIRECTORY "${PROJECT_SOURCE_DIR}")

  execute_process(
    COMMAND ${CMAKE_COMMAND} -E rename "${EXTRACTED}/CHANGELOG" "CHANGELOG"
    WORKING_DIRECTORY "${PROJECT_SOURCE_DIR}")

  execute_process(
    COMMAND ${CMAKE_COMMAND} -E rename "${EXTRACTED}/LICENSE" "LICENSE"
    WORKING_DIRECTORY "${PROJECT_SOURCE_DIR}")

  execute_process(
    COMMAND ${CMAKE_COMMAND} -E remove "${MY_DOWNLOAD_PATH}" 
    WORKING_DIRECTORY "${PROJECT_SOURCE_DIR}")

  execute_process(
    COMMAND ${CMAKE_COMMAND} -E remove_directory "${EXTRACTED}"
    WORKING_DIRECTORY "${PROJECT_SOURCE_DIR}")

  execute_process(
    COMMAND ${CMAKE_COMMAND} -E touch downloaded
    WORKING_DIRECTORY "${PROJECT_SOURCE_DIR}")

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
