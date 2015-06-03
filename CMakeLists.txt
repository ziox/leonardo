cmake_minimum_required(VERSION 2.8.3)
project(leonardo)

## C++11! WTF!
add_definitions(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  image_transport
  sensor_msgs
  cv_bridge
  geometry_msgs
  tf
  ardrone_autonomy
)

## Catkin

catkin_package()


## Build

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(simple_detector_node src/detector/main.cpp src/detector/simple_detector.cpp)
target_link_libraries(simple_detector_node
  ${catkin_LIBRARIES}
  aruco
)

add_executable(odometry_node src/odometry_node.cpp)
target_link_libraries(odometry_node
	${catkin_LIBRARIES}
)

add_executable(localization_node src/localization_node.cpp)
target_link_libraries(localization_node
	${catkin_LIBRARIES}
)

add_executable(path_planner_node src/path_planner_node.cpp)
target_link_libraries(path_planner_node
  ${catkin_LIBRARIES}
)

add_executable(schetto_ctrl_node src/controller_node.cpp)
target_link_libraries(schetto_ctrl_node
	${catkin_LIBRARIES}
)

add_executable(teleop_node src/teleop_node.cpp)
target_link_libraries(teleop_node
	${catkin_LIBRARIES}
)
