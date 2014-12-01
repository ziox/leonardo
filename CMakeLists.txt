cmake_minimum_required(VERSION 2.8.3)
project(leonardo)

## C++11! WTF!
add_definitions(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
)

## Catkin

catkin_package()


## Build

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(localizator src/localizator.cpp)
target_link_libraries(localizator
  ${catkin_LIBRARIES}
)

add_executable(navigator src/navigator.cpp)
target_link_libraries(navigator
  ${catkin_LIBRARIES}
)

add_executable(controller src/controller.cpp)
target_link_libraries(controller
  ${catkin_LIBRARIES}
)
