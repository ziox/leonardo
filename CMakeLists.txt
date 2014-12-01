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
