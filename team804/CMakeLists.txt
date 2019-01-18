cmake_minimum_required(VERSION 2.8.3)
project(team804)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  image_transport
  roscpp
  sensor_msgs
)

set(OpenCV_STATIC OFF)
find_package(  OpenCV REQUIRED )
include_directories(${OpenCV_INCLUDE_DIRS})

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES team804
  CATKIN_DEPENDS roscpp
  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(team804
  src/detectlane.cpp
  src/carcontrol.cpp
  src/detectsign.cpp
)

add_executable(team804_node src/main.cpp)

add_dependencies(team804_node ${catkin_EXPORTED_TARGETS})

target_link_libraries(
  team804_node
  team804
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
  ${Boost_LIBRARIES}
)


