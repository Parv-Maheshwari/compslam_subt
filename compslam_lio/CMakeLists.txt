cmake_minimum_required(VERSION 3.0.2)
project(compslam_lio)

## Compile as C++14, supported in ROS Kinetic and newer
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_FLAGS "-O3 -pthread")

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
  std_srvs
  nodelet
  message_filters
  compslam_lio_msgs
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

catkin_package(
  CATKIN_DEPENDS geometry_msgs nav_msgs roscpp rospy std_msgs tf pcl_conversions eigen_conversions tf_conversions nodelet message_filters sensor_msgs compslam_lio_msgs
  DEPENDS EIGEN3 PCL Boost
  INCLUDE_DIRS include
  LIBRARIES compslam_lio
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
target_link_libraries(multiScanRegistration compslam_lio)

add_executable(laserOdometry src/laser_odometry_node.cpp)
target_link_libraries(laserOdometry compslam_lio)

add_executable(laserMapping src/laser_mapping_node.cpp)
target_link_libraries(laserMapping compslam_lio)

add_executable(transformMaintenance src/transform_maintenance_node.cpp)
target_link_libraries(transformMaintenance compslam_lio )

add_dependencies(compslam_lio ${catkin_EXPORTED_TARGETS})

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
