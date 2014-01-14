cmake_minimum_required(VERSION 2.8.3)
project(pc_guided_grasp_planning)

# Find VTK
find_package(VTK)
if(VTK_FOUND)
 include (${VTK_USE_FILE})
endif()

find_package(PkgConfig)
pkg_check_modules(LIBXML++ REQUIRED libxml++-2.6)

FIND_PACKAGE ( OpenSceneGraph COMPONENTS osgSim osgUtil osgDB osgFX osgGA osgTerrain osgViewer osgText osgWidget osg)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  nav_msgs
  tf
  tf_conversions
  mar_core
  mar_ros_bridge
  mar_perception
  uwsim
)
find_package(uwsim REQUIRED)
find_package(uwsim_osgocean REQUIRED)
find_package(uwsim_osgworks REQUIRED)
find_package(uwsim_osgbullet REQUIRED)
find_package(uwsim_bullet REQUIRED)
find_package(VISP REQUIRED)
find_package(PCL REQUIRED)

catkin_package(
  CATKIN_DEPENDS  
    roscpp
    sensor_msgs
    nav_msgs
    tf
    tf_conversions
    mar_core
    mar_ros_bridge
    mar_perception
    uwsim
  DEPENDS
    libxml++-2.6
    OpenSceneGraph
    uwsim_osgocean
    uwsim_osgworks
    uwsim_osgocean
    uwsim_bullet
    VISP
    PCL
)
add_definitions(${VISP_DEFINITIONS})
INCLUDE_DIRECTORIES(${uwsim_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS} ${OPENSCENEGRAPH_INCLUDE_DIRS} ${LIBXML++_INCLUDE_DIRS} ${VISP_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS} ${BULLET_INCLUDE_DIRS} ${OSGWORKS_INCLUDE_DIR} ${OSGBULLET_INCLUDE_DIR} ${OSGOCEAN_INCLUDE_DIRS})
LINK_DIRECTORIES(${LIBXML++_LIBRARY_DIRS} ${VISP_LIBRARY_DIRS})

add_executable(arm5e_pc_grasp_planning src/arm5e_pc_grasp_planning.cpp)
target_link_libraries(arm5e_pc_grasp_planning ${catkin_LIBRARIES} ${OPENSCENEGRAPH_LIBRARIES} ${LIBXML++_LIBRARIES} ${PCL_LIBRARIES} vtkHybrid vtkRendering)

