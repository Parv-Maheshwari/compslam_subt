cmake_minimum_required(VERSION 3.0.2)
project(compslam_lio_msgs)

find_package(catkin REQUIRED COMPONENTS
  std_msgs
  sensor_msgs
  geometry_msgs
  message_generation
)

add_message_files(
  DIRECTORY msg
  FILES OptStatus.msg Submaps.msg
)

add_service_files(
  DIRECTORY
    srv
  FILES
    RequestSubmaps.srv
    ShareSubmaps.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
  sensor_msgs
  geometry_msgs
)

catkin_package(
  CATKIN_DEPENDS std_msgs geometry_msgs message_runtime
) 
