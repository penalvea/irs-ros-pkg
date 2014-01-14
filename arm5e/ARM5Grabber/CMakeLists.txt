cmake_minimum_required(VERSION 2.8.3)
project(ARM5Grabber)

find_package(catkin REQUIRED COMPONENTS roscpp sensor_msgs image_transport)

catkin_package(
  CATKIN_DEPENDS
    roscpp
    sensor_msgs
    image_transport
)
include_directories(${PROJECT_SOURCE_DIR}/include ${catkin_INCLUDE_DIRS})

ADD_DEFINITIONS(-DHAVE_LIBDV)
add_executable(ARM5Grabber src/main.cc src/dvframe.cc  src/error.cc  src/frame.cc  src/hdvframe.cc  src/iec13818-1.cc  src/iec13818-2.cc  src/ieee1394io.cc  src/raw1394util.c)
target_link_libraries(ARM5Grabber avc1394 iec61883 raw1394 rom1394 pthread dv ${catkin_LIBRARIES})

