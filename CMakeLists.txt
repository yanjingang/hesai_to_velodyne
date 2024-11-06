cmake_minimum_required(VERSION 3.0.2)
project(hesai_to_velodyne)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14 -O3")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  pcl_ros
  pcl_conversions

  std_msgs
  sensor_msgs
)

catkin_package(
  INCLUDE_DIRS 
  DEPENDS PCL
)

include_directories(
	include
	${catkin_INCLUDE_DIRS}
	${PCL_INCLUDE_DIRS}
)

link_directories(
	include
	${PCL_LIBRARY_DIRS}
)


add_executable(pandar40p_to_velodyne src/pandar40p_to_velodyne.cpp)
add_dependencies(pandar40p_to_velodyne ${catkin_EXPORTED_TARGETS})
target_link_libraries(pandar40p_to_velodyne ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBRARIES})
