cmake_minimum_required(VERSION 3.0.2)
project(zeus_watchdog)

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

add_executable(zeus_watchdog src/zeus_watchdog.cpp)
add_dependencies(zeus_watchdog zeus_watchdog_generate_messages_cpp)
target_link_libraries(zeus_watchdog ${catkin_LIBRARIES})