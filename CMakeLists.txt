cmake_minimum_required(VERSION 3.0.2)
project(steve_watchdog)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  topic_tools
  std_msgs
  geometry_msgs
  message_generation
)

add_message_files(
  FILES
  TopicStatus.msg
  TopicArray.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS message_runtime
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(steve_watchdog src/steve_watchdog.cpp)
add_dependencies(steve_watchdog steve_watchdog_generate_messages_cpp)
target_link_libraries(steve_watchdog ${catkin_LIBRARIES})