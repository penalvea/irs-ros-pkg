cmake_minimum_required(VERSION 2.8.3)
project(pruebas_deep_learning)


find_package(catkin REQUIRED COMPONENTS
  roscpp
)
find_package(PCL REQUIRED)






catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES pruebas_deep_learning
#  CATKIN_DEPENDS roscpp
  DEPENDS
  	PCL
)



include_directories(
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)


add_executable(generate_models src/generate_models.cpp)
target_link_libraries(generate_models ${catkin_LIBRARIES} ${PCL_LIBRARIES})

add_executable(rotate_pointcloud src/rotate_pointcloud.cpp)
target_link_libraries(rotate_pointcloud ${catkin_LIBRARIES} ${PCL_LIBRARIES})

add_executable(generate_dataset src/generate_dataset.cpp)
target_link_libraries(generate_dataset ${catkin_LIBRARIES} ${PCL_LIBRARIES})
