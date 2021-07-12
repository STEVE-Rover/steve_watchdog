cmake_minimum_required(VERSION 3.0.2)
project(steve_watchdog)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  topic_tools
  std_msgs
)

catkin_package(
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(steve_watchdog src/steve_watchdog.cpp)
target_link_libraries(steve_watchdog ${catkin_LIBRARIES})