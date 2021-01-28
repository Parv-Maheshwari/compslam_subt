add_library(loam
            math_utils.h
            ScanRegistration.cpp
            BasicScanRegistration.cpp
            MultiScanRegistration.cpp
            LaserOdometry.cpp
            BasicLaserOdometry.cpp
            LaserMapping.cpp
            BasicLaserMapping.cpp
            TransformMaintenance.cpp
            BasicTransformMaintenance.cpp
            laser_multiscan_registration_nodelet.cpp
            laser_odometry_nodelet.cpp
            laser_mapping_nodelet.cpp
            laser_transform_maintenance_nodelet.cpp)
target_link_libraries(loam ${catkin_LIBRARIES} ${PCL_LIBRARIES} gtsam gtsam_unstable)

install(TARGETS loam
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
