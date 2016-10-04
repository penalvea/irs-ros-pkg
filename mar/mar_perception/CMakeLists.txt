cmake_minimum_required(VERSION 2.8.3)
project(mar_perception)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  geometry_msgs
  visualization_msgs
  image_transport
  mar_core
  mar_robot_arm5e
  mar_msgs
  mar_ros_bridge
  esm_tracking
  visp_bridge
)
find_package(VISP REQUIRED)
find_package(PCL REQUIRED)
find_package(OpenCV REQUIRED)
find_package(orocos_kdl REQUIRED)

set (CMAKE_CXX_FLAGS "-fPIC")



catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS    
    geometry_msgs
    roscpp
    visualization_msgs
    image_transport
    mar_msgs
    mar_core
    mar_robot_arm5e
    mar_ros_bridge
    esm_tracking
    visp_bridge
  DEPENDS
    orocos_kdl
    VISP
    PCL
    OpenCV
)

#Library type definition needed for ESMlibrary (INRIA)
if( CMAKE_SIZEOF_VOID_P EQUAL 8)
  set( LIBRARY_TYPE STATIC)
else( CMAKE_SIZEOF_VOID_P EQUAL 8)	
  set( LIBRARY_TYPE SHARED )
endif( CMAKE_SIZEOF_VOID_P EQUAL 8)

add_definitions(${VISP_DEFINITIONS})
link_directories(${VISP_LIBRARY_DIRS} ${Boost_LIBRARY_DIRS} ${catkin_LIBRARY_DIRS})
include_directories(include ${catkin_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS} ${VISP_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS})

add_library(${PROJECT_NAME} ${LIBRARY_TYPE}
src/ESMTracking.cpp
src/ESMMRTracking.cpp
src/VirtualImage.cpp
src/VispUtils.cpp
src/PCLUtils.cpp
src/LaserPeakDetector.cpp
src/Reconstruction3D.cpp
src/ArmLaserReconstruction3DStatic.cpp
src/ArmLaserReconstruction3DEye.cpp
src/PCGraspPlanning.cpp
src/PCAutonomousGraspPlanning.cpp
src/MotionEstimator.cpp
src/PlanarDetector.cpp
)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBRARIES} ${VISP_LIBRARIES})


install(
  TARGETS
    ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}
   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
