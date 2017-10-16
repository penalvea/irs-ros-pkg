cmake_minimum_required(VERSION 2.8.3)
project(laser_grasp)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  mar_perception
  mar_robot_arm5e
  mar_ros_bridge
  roscpp
)
find_package(PCL REQUIRED)




catkin_package(
  INCLUDE_DIRS include
  LIBRARIES laser_grasp
  CATKIN_DEPENDS mar_perception mar_robot_arm5e mar_ros_bridge roscpp uwsim
  DEPENDS PCL
)


include_directories(
  ${PROJECT_SOURCE_DIR}/include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)
link_directories(${catkin_LIBRARY_DIRS})


add_library(laser_grasp
   src/laser_grasp.cpp
   src/cloud_registration.cpp
   src/best_grasp.cpp
   src/symmetry.cpp
 )
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES}  ${PCL_LIBRARIES})
add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS})




#add_executable(laser_grasp_node src/laser_grasp_node.cpp)
#add_dependencies(laser_grasp_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

# target_link_libraries(laser_grasp_node
#   ${catkin_LIBRARIES}
#   laser_grasp
# )

#add_executable(cloud_registration src/cloud_registration_node.cpp)
#add_dependencies(cloud_registration ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})#

#target_link_libraries(cloud_registration
#  ${catkin_LIBRARIES}
#  laser_grasp
#)

#add_executable(best_grasp src/best_grasp_node.cpp)
#add_dependencies(best_grasp ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

#target_link_libraries(best_grasp
#  ${catkin_LIBRARIES}
#  laser_grasp
#)

#add_executable(ros_to_pcl src/ros_to_pcl.cpp)
#add_dependencies(ros_to_pcl ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

#target_link_libraries(ros_to_pcl
#  ${catkin_LIBRARIES}
#  laser_grasp
#)

#add_executable(filter_cloud src/filter_cloud.cpp)
#add_dependencies(filter_cloud ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

#target_link_libraries(filter_cloud
#  ${catkin_LIBRARIES}
#  laser_grasp
#)

#add_executable(segment_cloud src/segment_cloud.cpp)
#add_dependencies(segment_cloud ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

#target_link_libraries(segment_cloud
#  ${catkin_LIBRARIES}
#  laser_grasp
#)


add_executable(symmetry src/symmetry_node.cpp)
add_dependencies(symmetry ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(symmetry
  ${catkin_LIBRARIES}
  laser_grasp
)
