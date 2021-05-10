cmake_minimum_required(VERSION 2.8.3)
project(loam)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  sensor_msgs
  roscpp
  rospy
  std_msgs
  tf
  pcl_conversions
  eigen_conversions
  tf_conversions
  message_generation
  std_srvs
  nodelet
  message_filters
)

find_package(Eigen3 REQUIRED)
message("Eigen Version:" ${EIGEN3_VERSION_STRING})
message("Eigen Path:" ${Eigen3_DIR})

find_package(PCL REQUIRED)
message("PCL Version:" ${PCL_VERSION})
message("PCL Path:" ${PCL_DIR})

find_package(GTSAM REQUIRED)
message("GTSAM version: ${GTSAM_VERSION}")
message("GTSAM path: ${GTSAM_DIR}")

find_package(Boost REQUIRED COMPONENTS timer)
message("Boost version: ${Boost_VERSION}")

include_directories(
  include
	${catkin_INCLUDE_DIRS}
	#${EIGEN3_INCLUDE_DIR}
	${PCL_INCLUDE_DIRS}
  ${GTSAM_INCLUDE_DIR}
  ${Boost_INCLUDE_DIRS}
)

add_message_files(
  DIRECTORY msg
  FILES
  OptStatus.msg
  Graph.msg
  GraphNode.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

catkin_package(
  CATKIN_DEPENDS geometry_msgs nav_msgs roscpp rospy std_msgs tf pcl_conversions eigen_conversions tf_conversions message_runtime nodelet message_filters
  DEPENDS EIGEN3 PCL Boost
  INCLUDE_DIRS include
  LIBRARIES loam
)

## Compile as C++14, supported in ROS Kinetic and newer
# set_property(TARGET invz_player PROPERTY CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Turn on if needed otherwise makes multiScanRegistration node crash
# if(NOT CMAKE_CROSSCOMPILING)
#   add_definitions(-march=native)
# endif()

add_subdirectory(src/lib)

add_executable(multiScanRegistration src/multi_scan_registration_node.cpp)
target_link_libraries(multiScanRegistration loam)

add_executable(laserOdometry src/laser_odometry_node.cpp)
target_link_libraries(laserOdometry loam)

add_executable(laserMapping src/laser_mapping_node.cpp)
target_link_libraries(laserMapping loam)

add_executable(transformMaintenance src/transform_maintenance_node.cpp)
target_link_libraries(transformMaintenance loam )

add_dependencies(loam ${catkin_EXPORTED_TARGETS})

install(TARGETS multiScanRegistration
                laserOdometry
                laserMapping
                transformMaintenance
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark cpp header files for installation
install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
