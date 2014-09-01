cmake_minimum_required(VERSION 2.8.3)
project(mar_action)

find_package(catkin REQUIRED COMPONENTS mar_core mar_perception)
find_package(VISP REQUIRED)

catkin_package(
  INCLUDE_DIRS include  
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS
    mar_core
    mar_perception  
  DEPENDS
    VISP
    
)

#Library type definition needed for ESMlibrary (INRIA)
if( CMAKE_SIZEOF_VOID_P EQUAL 8)
  set( LIBRARY_TYPE STATIC)
else( CMAKE_SIZEOF_VOID_P EQUAL 8)	
  set( LIBRARY_TYPE SHARED )
endif( CMAKE_SIZEOF_VOID_P EQUAL 8)

add_definitions(${VISP_DEFINITIONS})
link_directories(${VISP_LIBRARY_DIRS})
include_directories(${PROJECT_SOURCE_DIR}/include ${catkin_INCLUDE_DIRS} ${VISP_INCLUDE_DIRS})

add_library(${PROJECT_NAME} ${LIBRARY_TYPE} src/Reconstruction3DAction.cpp)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

install(
  TARGETS
    ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}
   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

