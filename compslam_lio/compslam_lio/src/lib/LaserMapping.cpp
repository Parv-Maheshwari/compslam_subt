// Copyright 2013, Ji Zhang, Carnegie Mellon University
// Further contributions copyright (c) 2016, Southwest Research Institute
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// 1. Redistributions of source code must retain the above copyright notice,
//    this list of conditions and the following disclaimer.
// 2. Redistributions in binary form must reproduce the above copyright notice,
//    this list of conditions and the following disclaimer in the documentation
//    and/or other materials provided with the distribution.
// 3. Neither the name of the copyright holder nor the names of its
//    contributors may be used to endorse or promote products derived from this
//    software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
//
// This is an implementation of the algorithm described in the following paper:
//   J. Zhang and S. Singh. LOAM: Lidar Odometry and Mapping in Real-time.
//     Robotics: Science and Systems Conference (RSS). Berkeley, CA, July 2014.

#include "compslam_lio/LaserMapping.h"

#include <tf/transform_listener.h>

#include "compslam_lio/common.h"

namespace loam {

bool LaserMapping::setup(ros::NodeHandle& node, ros::NodeHandle& privateNode) {
  // fetch laser mapping params
  double dParam;       //double parameter
  float fParam;        //float parameter
  int iParam;          //integer parameter
  bool bParam;         //bool parameter
  std::string sParam;  //string parameter

  if (privateNode.getParam("scanPeriod", fParam)) {
    if (fParam <= 0) {
      ROS_ERROR("Invalid scanPeriod parameter: %f (expected > 0)", fParam);
      return false;
    } else {
      setScanPeriod(fParam);
      ROS_INFO("Set scanPeriod: %g", fParam);
    }
  }

  if (privateNode.getParam("mapMaxIterations", iParam)) {
    if (iParam < 1) {
      ROS_ERROR("Invalid mapMaxIterations parameter: %d (expected > 0)", iParam);
      return false;
    } else {
      setMaxIterations(iParam);
      ROS_INFO("Set mapMaxIterations: %d", static_cast<int>(maxIterations()));
    }
  }

  if (privateNode.getParam("mapMinIterations", iParam)) {
    if (iParam < 0) {
      ROS_ERROR("Invalid mapMinIterations parameter: %d", iParam);
      return false;
    } else {
      setMinIterations(iParam);
      ROS_INFO("Set mapMinIterations: %d", static_cast<int>(minIterations()));
    }
  }

  if (privateNode.getParam("mapDeltaTAbort", fParam)) {
    if (fParam <= 0) {
      ROS_ERROR("Invalid mapDeltaTAbort parameter: %f (expected > 0)", fParam);
      return false;
    } else {
      setDeltaTAbort(fParam);
      ROS_INFO("Set mapDeltaTAbort: %g", fParam);
    }
  }

  if (privateNode.getParam("mapDeltaRAbort", fParam)) {
    if (fParam <= 0) {
      ROS_ERROR("Invalid mapDeltaRAbort parameter: %f (expected > 0)", fParam);
      return false;
    } else {
      setDeltaRAbort(fParam);
      ROS_INFO("Set mapDeltaRAbort: %g", fParam);
    }
  }

  if (privateNode.getParam("cornerFilterSize", fParam)) {
    if (fParam < 0.001) {
      ROS_ERROR("Invalid cornerFilterSize parameter: %f (expected >= 0.001)", fParam);
      return false;
    } else {
      downSizeFilterCorner().setLeafSize(fParam, fParam, fParam);
      ROS_INFO("Set corner down size filter leaf size: %g", fParam);
    }
  }

  if (privateNode.getParam("surfaceFilterSize", fParam)) {
    if (fParam < 0.001) {
      ROS_ERROR("Invalid surfaceFilterSize parameter: %f (expected >= 0.001)", fParam);
      return false;
    } else {
      downSizeFilterSurf().setLeafSize(fParam, fParam, fParam);
      ROS_INFO("Set surface down size filter leaf size: %g", fParam);
    }
  }

  if (privateNode.getParam("mapVisFilterSize", fParam)) {
    if (fParam < 0.001) {
      ROS_ERROR("Invalid mapVisFilterSize parameter: %f (expected >= 0.001)", fParam);
      return false;
    } else {
      downSizeFilterMapVis().setLeafSize(fParam, fParam, fParam);
      ROS_INFO("Set map visualization down size filter leaf size: %g", fParam);
    }
  }

  if (privateNode.getParam("submapVisCropRange", fParam)) {
    if (fParam < 0.0) {
      ROS_ERROR("Invalid submapVisCropRange parameter: %f (expected >= 0.0)", fParam);
      return false;
    } else if (fParam == 0.0)
      ROS_INFO("Full submap visualized - Cropbox disabled");
    else {
      setSubmapVisCropRange(fParam);
      ROS_INFO("Set submap visualization cropbox range: %g", fParam);
    }
  }

  if (privateNode.getParam("rndSampleMapVisNoOfSamples", iParam)) {
    if (iParam < 0) {
      ROS_ERROR("Invalid rndSampleMapVisNoOfSamples parameter: %d (expected >= 0)", iParam);
      return false;
    }
    else {
      rndSampleFilterMapVis().setSample(iParam);
      ROS_INFO("Set number of Samples used for random sampling the visualized Pointcloud: %d", iParam);
    }
  }

  //CUSTOMIZATION
  //Set LOAM internal Map Cube Size
  if (privateNode.getParam("mapCubeSize", dParam)) {
    if (dParam < 1.0) {
      ROS_ERROR("Invalid mapCubeSize parameter: %f (expected > 1.0)", dParam);
      return false;
    } else {
      setMapCubeSize(dParam);
      ROS_INFO("Set mapCubeSize: %f", dParam);
    }
  }

  //Set LOAM internal Map Dimension WxDxH in Cube units
  std::vector<int> mapWHD{21, 11, 21};
  if (privateNode.getParam("mapDimensionsInCubes", mapWHD)) {
    if (mapWHD[0] < 3 || mapWHD[1] < 3 || mapWHD[2] < 3) {
      ROS_ERROR("Invalid Map Dimensions in Cubes");
      return false;
    } else {
      setMapWidthInCubes(mapWHD[0]);
      setMapHeightInCubes(mapWHD[1]);
      setMapDepthInCubes(mapWHD[2]);
      ROS_INFO_STREAM("Set Map Dimension in Cubes (WxHxD): " << getMapWidthInCubes() << "x" << getMapHeightInCubes() << "x" << getMapDepthInCubes());
    }
  }

  //Set LOAM internal Map robot start location in Cube Units
  std::vector<int> mapStartWHD{10, 5, 10};
  if (privateNode.getParam("mapStartLocationInCubes", mapStartWHD)) {
    if (mapStartWHD[0] < 1 || mapStartWHD[0] > getMapWidthInCubes() ||
        mapStartWHD[1] < 1 || mapStartWHD[1] > getMapHeightInCubes() ||
        mapStartWHD[2] < 1 || mapStartWHD[2] > getMapDepthInCubes()) {
      ROS_ERROR("Invalid Map Starting Location in Cubes");
      return false;
    } else {
      setMapStartLocWidthInCubes(mapStartWHD[0]);
      setMapStartLocHeightInCubes(mapStartWHD[1]);
      setMapStartLocDepthInCubes(mapStartWHD[2]);
      ROS_INFO_STREAM("Set Map Start Location in Cubes (WxHxD): " << mapStartWHD[0] << "x" << mapStartWHD[1] << "x" << mapStartWHD[2]);
    }
  }

  //Set number of internal neigboring cubes used for creating submap for pcl to map matching
  if (privateNode.getParam("numNeighborSubmapCubes", iParam)) {
    if (iParam < 1.0) {
      ROS_ERROR("Invalid Number of Neighbor Cubes used for Submap Creation: %d (expected >= 1.0)", iParam);
      return false;
    } else {
      setNumNeighborSubmapCubes(iParam);
      ROS_INFO("Set Number of Neighbor Cubes used for Submap Creation: %d", iParam);
    }
  }

  //Set Degeneracy detection threshold for scan-to-map matching
  if (privateNode.getParam("mapDegenEigVal", fParam)) {
    if (fParam <= 0) {
      ROS_ERROR("Invalid mapDegenEigVal parameter: %f (expected > 0)", fParam);
      return false;
    } else {
      setDegenEigenThreshold(fParam);
      ROS_INFO("Set mapDegenEigVal: %g", fParam);
    }
  }

  //Set Max Robot Motion threshold for ignoring erroneous priors
  if (privateNode.getParam("mapPriorNormThresh", dParam)) {
    if (dParam <= 0) {
      ROS_ERROR("Invalid mapPriorNormThresh parameter: %f (expected > 0)", dParam);
      return false;
    } else {
      setExtOdomPriorNormThreshold(dParam);
      ROS_INFO("Set mapPriorNormThresh: %g", dParam);
    }
  }

  //File path for reading and saving submaps
  std::string filePath;
  privateNode.getParam("saved_submaps_filepath", filePath);
  if (!filePath.empty()) {
    filePath += "/";
    ROS_INFO_STREAM("Set saved_submaps_filepath: " << filePath);
    setSavedSubmapFilepath(filePath);
  } else
    ROS_WARN("No filepath specified to Save/Load Submaps");

  //Load saved submap for initialization
  bParam = false;
  privateNode.getParam("useSavedSubmapsForInitialization", bParam);
  //Use Saved Submaps
  ROS_INFO_STREAM("Set useSavedSubmapsForInitialization: " << (bParam ? "True" : "False"));
  if (bParam) {
    if (!filePath.empty()) {
      //Use Saved Submaps
      setSavedSubmapUse(bParam);
      //Use position guess for Submap localization
      std::vector<float> fSubmapLocGuess{0, 0, 0, 0, 0, 0};  //x,y,z,yaw,pitch,roll
      if (privateNode.getParam("submapLocalizationInitGuess", fSubmapLocGuess))
        if (fSubmapLocGuess.size() == 6)
          setSubmapLocalizationGuess(fSubmapLocGuess);
        else
          ROS_WARN("LOAM - Incorrect input size for submap localization guess - Expected XYZ in meters and YPR in radians");
    } else
      ROS_ERROR("LOAM - Use of Saved Submaps is enabled but no filepath is defined");
  }

  //External submaps for co-localization at initialization
  bParam = false;
  if (privateNode.getParam("waitAndUseExternalSubmapsForInit", bParam)) {
    if (bParam) {
      ROS_WARN("LaserMapping - Using External Submaps and Pose for Initlization and Co-localization - Waiting for trigger through service call ...");

      //Check if name of service call to acquire external maps from is provided
      privateNode.getParam("submapsRequestServiceCall", _submapsRequestSrvCall);
      //Check if valid string
      if (!_submapsRequestSrvCall.empty())
        ROS_INFO_STREAM("LaserMapping - External Submaps Service call: " << _submapsRequestSrvCall);
      else {
        ROS_ERROR("LaserMapping - External Submaps Usage set to true but NO service call provided - DIABLING EXTERNAL SUBMAP USE");
        bParam = false;
      }

      //Set external submaps usage
      useExternalSubmapsForInit(bParam);

      //Check if origin or current external pose should be used as co-locallization initial guess
      bParam = false;
      privateNode.getParam("useExternalSubmapOriginForInit", bParam);
      useExternalSubmapOriginPoseForInit(bParam);
      if (bParam)
        ROS_WARN("LaserMapping - Using Submap ORIGIN pose to initialize co-localizatin in external submaps");
      else
        ROS_INFO("LaserMapping - Using shared pose to initialize co-localizatin in external submaps");
    } else
      ROS_INFO("LaserMapping - Use External Submaps and Pose for Initlization and Co-localization: False");
  }

  //External Prior
  if (privateNode.getParam("extPriorAvailable", bParam)) {
    setExternalPriorStatus(bParam);
    ROS_INFO_STREAM("LaserMapping - External Prior Available: " << (bParam ? "True" : "False"));
    if (bParam) {
      //Set external ODOMETRY frame
      if (privateNode.getParam("extOdomFrame", sParam)) {
        ROS_INFO_STREAM("LaserMapping - external ODOMETRY frame: " << sParam);
        setExternalOdometryFrame(sParam);
      } else {
        ROS_WARN("LaserMapping - external ODOMETRY frame not set - DISABLING USE OF EXTERNAL PRIOR");
        setExternalPriorStatus(false);
      }

      //Set external FIXED frame
      if (privateNode.getParam("extFixedFrame", sParam)) {
        ROS_INFO_STREAM("LaserMapping - external FIXED frame: " << sParam);
        setExternalFixedFrame(sParam);
      } else {
        ROS_WARN("LaserMapping - external FIXED frame not set - DISABLING USE OF EXTERNAL PRIOR");
        setExternalPriorStatus(false);
      }

      //Set external SENSOR frame
      if (privateNode.getParam("extSensorFrame", sParam)) {
        ROS_INFO_STREAM("LaserMapping - external SENSOR frame: " << sParam);
        setExternalSensorFrame(sParam);
      } else {
        ROS_WARN("LaserMapping - external SENSOR frame not set - DISABLING USE OF EXTERNAL PRIOR");
        setExternalPriorStatus(false);
      }

      //Set external odometry time offset
      dParam = 0.0;
      if (privateNode.getParam("extOdomTimeOffset", dParam))
        setExternalOdometryTimeOffset(dParam);
      ROS_INFO_STREAM("LaserMapping - external odometry time offset[s]: " << dParam);
    }
  }

  //Fallback External Prior
  if (privateNode.getParam("fallbackExtPriorAvailable", bParam)) {
    setFallbackExternalPriorStatus(bParam);
    ROS_INFO_STREAM("LaserMapping - Fallback External Prior Available: " << (bParam ? "True" : "False"));
    if (bParam) {
      //Set external ODOMETRY frame
      if (privateNode.getParam("fallbackExtOdomFrame", sParam)) {
        ROS_INFO_STREAM("LaserMapping - fallback external ODOMETRY frame: " << sParam);
        setFallbackExternalOdometryFrame(sParam);
      } else {
        ROS_WARN("LaserMapping - fallback external ODOMETRY frame not set - DISABLING USE OF FALLBACK EXTERNAL PRIOR");
        setFallbackExternalPriorStatus(false);
      }

      //Set external FIXED frame
      if (privateNode.getParam("fallbackExtFixedFrame", sParam)) {
        ROS_INFO_STREAM("LaserMapping - fallback external FIXED frame: " << sParam);
        setFallbackExternalFixedFrame(sParam);
      } else {
        ROS_WARN("LaserMapping - fallback external FIXED frame not set - DISABLING USE OF FALLBACK EXTERNAL PRIOR");
        setFallbackExternalPriorStatus(false);
      }

      //Set external SENSOR frame
      if (privateNode.getParam("fallbackExtSensorFrame", sParam)) {
        ROS_INFO_STREAM("LaserMapping - fallback external SENSOR frame: " << sParam);
        setFallbackExternalSensorFrame(sParam);
      } else {
        ROS_WARN("LaserMapping - fallback external SENSOR frame not set - DISABLING USE OF FALLBACK EXTERNAL PRIOR");
        setFallbackExternalPriorStatus(false);
      }

      //Set fallback external odometry time offset
      dParam = 0.0;
      if (privateNode.getParam("fallbackExtOdomTimeOffset", dParam))
        setFallbackExternalOdomTimeOffset(dParam);
      ROS_INFO_STREAM("LaserMapping - fallback external odometry time offset[s]: " << dParam);
    }
  }

  //Set LiDAR frame
  if (privateNode.getParam("lidarFrame", sParam)) {
    ROS_INFO_STREAM("LaserMapping - LiDAR frame: " << sParam);
    setLidarFrame(sParam);
  } else {
    ROS_WARN("LaserMapping - LiDAR frame not set - DISABLING USE OF EXTERNAL PRIORS");
    setExternalPriorStatus(false);
    setFallbackExternalPriorStatus(false);
  }

  //Set if external frame should be used for gravity rotation alignment
  if (privateNode.getParam("useExtRotationAlignment", bParam)) {
    if (bParam) {
      ROS_WARN_STREAM("LaserMapping - Use External Fixed Frame to Fix Map Frame Gravity Alignment");
      useExtRotationAlignment(bParam);
    } else
      ROS_INFO_STREAM("LaserMapping - External Fixed Frame for Gravity Alignment - DISABLED");
  }

  //Set input PCL rotation w.r.t LiDAR frame
  if (privateNode.getParam("rotateInputCloud", bParam)) {
    setInputPCLRotationStatus(bParam);
    ROS_INFO_STREAM("LaserMapping - Rotate Input Pointcloud before using: " << (bParam ? "True" : "False"));
    if (bParam) {
      std::vector<double> ypr{0.0, 0.0, 0.0};
      if (privateNode.getParam("inputCloudRotation", ypr)) {
        if (ypr.size() == 3) {
          Eigen::Quaterniond q = Eigen::AngleAxisd(ypr[0], Eigen::Vector3d::UnitZ()) *
                                 Eigen::AngleAxisd(ypr[1], Eigen::Vector3d::UnitY()) *
                                 Eigen::AngleAxisd(ypr[2], Eigen::Vector3d::UnitX());
          q.normalize();
          tf::Transform T(tf::Quaternion(q.x(), q.y(), q.z(), q.w()), tf::Vector3(0, 0, 0)); /*quaternion order: x,y,z,w*/
          setLiDARtoInputPCLRotation(T);
          ROS_INFO_STREAM("LaserMapping - Rotation applied to input clound in order YPR(rad): [" << ypr[0] << " " << ypr[1] << " " << ypr[2] << "]");
        } else {
          ROS_ERROR("LaserMapping - Expected order Yaw-Pitch-Roll in radians");
          setInputPCLRotationStatus(false);
        }
      }
    }
  }

  //Force External FULL Prior/Motion Estimate instead of LaserOdometry Translation Prior
  if (privateNode.getParam("forceExtPriorUseMapping", bParam)) {
    forceExternalPriorUseForMapping(bParam);
    ROS_INFO_STREAM("LaserMapping - Force FULL External Prior Estimate Use: " << (bParam ? "True" : "False"));
  } else
    ROS_INFO_STREAM("LaserMapping - Using LaserOdometry Translation prior");

  //Force Path Publishing
  if (privateNode.getParam("forcePathPublishing", bParam)) {
    _forcePathPublishing = bParam;
    ROS_INFO_STREAM("LaserMapping - Force Path Publishing: " << (_forcePathPublishing ? "True" : "False"));
  }

  //Map Publish Rate Divider
  if (privateNode.getParam("mapPublishRateDivider", iParam)) {
    _mapPublishRateDivider = iParam;
    int ioRatio = 0;
    privateNode.getParam("ioRatio", ioRatio);
    ROS_INFO_STREAM("LaserMapping - Full Map Published at: " << (1.0 / (scanPeriod() * float(ioRatio) * float(_mapPublishRateDivider))) << " Hz");
  }

  //Verbose Ouput
  iParam = 0;
  if (privateNode.getParam("compslamLioVerbosity", iParam)) {
    setVerboseLevel(iParam);
    ROS_INFO("Set compslamLioVerbosity: %d", iParam);
  } else
    setVerboseLevel(0);
  //CUSTOMIZATION

  // advertise laser mapping topics
  _pubLaserCloudFullRes = node.advertise<sensor_msgs::PointCloud2>("registered_cloud", 2);
  _pubOdomAftMapped = node.advertise<nav_msgs::Odometry>("odom_aft_mapped_to_init", 5);
  _pubOdomAftMapped_CORRECTED = node.advertise<nav_msgs::Odometry>("odom_aft_mapped_to_init_CORRECTED", 5);
  _pubAftMappedTransformStamped_CORRECTED = node.advertise<geometry_msgs::TransformStamped>("transform_aft_mapped_to_init_CORRECTED", 5);

  // subscribe input Cloud and Odometry topics using ROS exact sync policy in a single callback
  _subCornerCloud.subscribe(node, "laser_cloud_corner_last", 2);
  _subFlatCloud.subscribe(node, "laser_cloud_surf_last", 2);
  _subFullCloud.subscribe(node, "velodyne_cloud_3", 2);
  _subLaserOdom.subscribe(node, "laser_odom_to_init", 2);
  _pclExactSyncPtr.reset(new message_filters::Synchronizer<_pclExactSyncPolicy>(_pclExactSyncPolicy(10), _subCornerCloud, _subFlatCloud, _subFullCloud, _subLaserOdom));
  _pclExactSyncPtr->registerCallback(boost::bind(&LaserMapping::syncInputsHandler, this, _1, _2, _3, _4));

  // subscribe to IMU topic
  _subImu = node.subscribe<sensor_msgs::Imu>("imu/data", 50, &LaserMapping::imuHandler, this);

  //Get node namespace for tf and remove leading "/" and add lagging "/" if namespace is not empty
  std::string tf_ns = node.getNamespace();
  if(!tf_ns.empty()) {
      tf_ns = tf_ns.erase(0, 1) + "/";
  }

  // initialize mapping tf messages
  _aftMappedTrans.frame_id_ = tf_ns + "camera_init";
  _aftMappedTrans.child_frame_id_ = tf_ns + "aft_mapped";

  // ARL Coordinate Frame Facilitators
  _gm_aftMappedTransformStamped_CORRECTED.header.frame_id = tf_ns + "camera_init_CORRECTED";
  _gm_aftMappedTransformStamped_CORRECTED.child_frame_id = tf_ns + "aft_mapped_to_init_CORRECTED";

  // Initialize path message and publisher
  _pathMsg.header.frame_id = _gm_aftMappedTransformStamped_CORRECTED.header.frame_id;
  _pathMsg.poses.reserve(20000);  //1 hour path @ 5Hz ~ 18000
  _pubPath = node.advertise<nav_msgs::Path>("laser_mapping_path", 5);

  // Publish optimization status
  _pubOptimizationStatus = node.advertise<compslam_lio_msgs::OptStatus>("laser_mapping_optimization_status", 5);
  // END ARL Coordinate Frame Facilitators

  //CUSTOMIZATION
  _subLaserOdometryStatus = node.subscribe<std_msgs::Header>("laser_odom_status", 5, &LaserMapping::laserOdometryStatusHandler, this);
  _srvSaveSubmapsToFile = privateNode.advertiseService("save_submaps_to_file", &LaserMapping::saveSubMapsToFile, this);
  _srvShareSubmaps = privateNode.advertiseService("share_submaps", &LaserMapping::shareSubmaps, this);
  _srvRequestSubmaps = privateNode.advertiseService("request_submaps", &LaserMapping::requestSubmaps, this);
  _pubMap = node.advertise<sensor_msgs::PointCloud2>("laser_map", 1);
  _pubSubmap = node.advertise<sensor_msgs::PointCloud2>("laser_submap", 1);
  _pubSubMapSearchCloud = node.advertise<sensor_msgs::PointCloud2>("submap_search_cloud", 1);
  //DEBUG
  _pubCurrentCornerSubmap = node.advertise<sensor_msgs::PointCloud2>("current_corner_submap", 1);
  _pubCurrentSurfSubmap = node.advertise<sensor_msgs::PointCloud2>("current_surf_submap", 1);
  _pubSavedCornerSubmap = node.advertise<sensor_msgs::PointCloud2>("saved_corner_submap", 1);
  _pubSavedSurfSubmap = node.advertise<sensor_msgs::PointCloud2>("saved_surf_submap", 1);
  //DEBUG

  //Reset BasicLaserMapping Map parameters and containers
  resetMapParamsAndContainers();
  //CUSTOMIZATION

  return true;
}

void LaserMapping::laserCloudCornerLastHandler(const sensor_msgs::PointCloud2ConstPtr& cornerPointsLastMsg) {
  _timeLaserCloudCornerLast = cornerPointsLastMsg->header.stamp;
  laserCloudCornerLast().clear();
  pcl::fromROSMsg(*cornerPointsLastMsg, laserCloudCornerLast());
}

void LaserMapping::laserCloudSurfLastHandler(const sensor_msgs::PointCloud2ConstPtr& surfacePointsLastMsg) {
  _timeLaserCloudSurfLast = surfacePointsLastMsg->header.stamp;
  laserCloudSurfLast().clear();
  pcl::fromROSMsg(*surfacePointsLastMsg, laserCloudSurfLast());
}

void LaserMapping::laserCloudFullResHandler(const sensor_msgs::PointCloud2ConstPtr& laserCloudFullResMsg) {
  _timeLaserCloudFullRes = laserCloudFullResMsg->header.stamp;
  laserCloud().clear();
  pcl::fromROSMsg(*laserCloudFullResMsg, laserCloud());
}

void LaserMapping::laserOdometryHandler(const nav_msgs::Odometry::ConstPtr& laserOdometry) {
  _timeLaserOdometry = laserOdometry->header.stamp;

  double roll, pitch, yaw;
  geometry_msgs::Quaternion geoQuat = laserOdometry->pose.pose.orientation;
  tf::Matrix3x3(tf::Quaternion(geoQuat.z, -geoQuat.x, -geoQuat.y, geoQuat.w)).getRPY(roll, pitch, yaw);  //smk: odometry coming in is still in LOAM coordinates

  updateOdometry(-pitch, -yaw, roll,
                 laserOdometry->pose.pose.position.x,
                 laserOdometry->pose.pose.position.y,
                 laserOdometry->pose.pose.position.z);
}

void LaserMapping::imuHandler(const sensor_msgs::Imu::ConstPtr& imuIn) {
  double roll, pitch, yaw;
  tf::Quaternion orientation;
  tf::quaternionMsgToTF(imuIn->orientation, orientation);
  tf::Matrix3x3(orientation).getRPY(roll, pitch, yaw);
  updateIMU({fromROSTime(imuIn->header.stamp), roll, pitch});  //smk: in mapping only roll and pitch of IMU are being used not the full Pose as in ScanRegistration
}

//CUSTOMIZATION
void LaserMapping::laserOdometryStatusHandler(const std_msgs::HeaderConstPtr& statusPtr) {
  if (statusPtr->seq > 0)
    setlaserOdometryStatus(true);  //Laser Odometry is Degenerate
  else
    setlaserOdometryStatus(false);
}

bool LaserMapping::saveSubMapsToFile(std_srvs::Empty::Request& /*request*/, std_srvs::Empty::Response& /*response*/) {
  std::string map_save_path = getSavedSubmapFilepath();
  pcl::io::savePCDFileASCII((map_save_path + "savedCornerSubmap.pcd"), getCurrentCornerSubmap());
  pcl::io::savePCDFileASCII((map_save_path + "savedSurfSubmap.pcd"), getCurrentSurfSubmap());
  ROS_WARN_STREAM("SAVING LOAM FEATURE SUBMAPS OF INITIAL SECTOR TO FILE at path: " << map_save_path);
  return true;
}

bool LaserMapping::shareSubmaps(compslam_lio_msgs::ShareSubmaps::Request& /*request*/, compslam_lio_msgs::ShareSubmaps::Response& resp) {
  if (BasicLaserMapping::getSubmapsAndPose(resp.Submaps)) {
    auto& m = resp.Submaps;
    ROS_INFO_STREAM("Shared Sharp/Flat submaps: " << m.sharp_submaps.size()
                                                  << ", points in last submaps: " << m.sharp_submaps[m.sharp_submaps.size() - 1].width << "/"
                                                  << m.flat_submaps[m.flat_submaps.size() - 1].width
                                                  << ", last cube indices: " << m.sharp_submaps[m.sharp_submaps.size() - 1].header.seq << "/"
                                                  << m.flat_submaps[m.flat_submaps.size() - 1].header.seq);
    return true;
  } else
    return false;
}

bool LaserMapping::requestSubmaps(compslam_lio_msgs::RequestSubmaps::Request& req, compslam_lio_msgs::RequestSubmaps::Response& resp) {
  //Call submap share service
  std::string submapsRequestSrvcall = req.anymal_prefix + _submapsRequestSrvCall;
  ROS_INFO_STREAM("LaserMapping - requestSubmaps: Calling service: " + submapsRequestSrvcall);
  compslam_lio_msgs::ShareSubmaps srv;
  if (ros::service::call<compslam_lio_msgs::ShareSubmaps>(submapsRequestSrvcall, srv)) {
    //Set submaps in respective cubes
    if (BasicLaserMapping::setSubmapsAndPose(srv.response.Submaps)) {
      auto& m = srv.response.Submaps;
      resp.message = "LaserMapping - Received Sharp/Flat submaps: " + std::to_string(m.sharp_submaps.size()) + ", points in last submaps: " + std::to_string(m.sharp_submaps[m.sharp_submaps.size() - 1].width) + "/" + std::to_string(m.flat_submaps[m.flat_submaps.size() - 1].width) + ", last cube index: " + std::to_string(m.sharp_submaps[m.sharp_submaps.size() - 1].header.seq);
    } else
      resp.message = "LaserMapping - Client could not PROCESS shared submaps - FAILED";
  } else
    resp.message = "LaserMapping - Client request to SHARE submaps - FAILED";
  return resp.success = true;
}

void LaserMapping::syncInputsHandler(const sensor_msgs::PointCloud2ConstPtr& pclCorner,
                                     const sensor_msgs::PointCloud2ConstPtr& pclFlat,
                                     const sensor_msgs::PointCloud2ConstPtr& pclFull,
                                     const nav_msgs::OdometryConstPtr& laserOdom) {
  //Process incoming pointclouds and odometry
  laserCloudCornerLastHandler(pclCorner);
  laserCloudSurfLastHandler(pclFlat);
  laserCloudFullResHandler(pclFull);
  laserOdometryHandler(laserOdom);

  // try processing new data
  process();
}
//CUSTOMIZATION

void LaserMapping::process() {
  //Update Timestamp of BasicLaserMapping should be same as _timeSurfPointsLessFlat from LaserOdometry
  updatePCLTime(_timeLaserOdometry);

  if (!BasicLaserMapping::process(fromROSTime(_timeLaserOdometry)))
    return;

  publishResult();
}

void LaserMapping::publishResult() {
  // publish transformed full resolution input cloud
  if(_pubLaserCloudFullRes.getNumSubscribers() > 0){
    transformFullResToMap();
    publishCloudMsg(_pubLaserCloudFullRes, laserCloud(), _timeLaserOdometry, _aftMappedTrans.frame_id_);
  }

  //CUTOMIZATION - DEBUG
  //Curent Submaps
  if (_pubCurrentCornerSubmap.getNumSubscribers() > 0)
    publishCloudMsg(_pubCurrentCornerSubmap, getCurrentCornerSubmap(), _timeLaserOdometry, _aftMappedTrans.frame_id_);
  if (_pubCurrentSurfSubmap.getNumSubscribers() > 0)
    publishCloudMsg(_pubCurrentSurfSubmap, getCurrentSurfSubmap(), _timeLaserOdometry, _aftMappedTrans.frame_id_);
  //Saved Submaps
  if (_pubSavedCornerSubmap.getNumSubscribers() > 0)
    publishCloudMsg(_pubSavedCornerSubmap, getSavedCornerSubmap(), _timeLaserOdometry, _aftMappedTrans.frame_id_);
  if (_pubSavedSurfSubmap.getNumSubscribers() > 0)
    publishCloudMsg(_pubSavedSurfSubmap, getSavedSurfSubmap(), _timeLaserOdometry, _aftMappedTrans.frame_id_);
  //CUTOMIZATION -DEBUG

  // publish odometry after mapped transformations
  geometry_msgs::Quaternion geoQuat = tf::createQuaternionMsgFromRollPitchYaw(transformAftMapped().rot_z.rad(), -transformAftMapped().rot_x.rad(), -transformAftMapped().rot_y.rad());

  nav_msgs::OdometryPtr odomAftMappedPtr(new nav_msgs::Odometry);
  odomAftMappedPtr->header.frame_id = _aftMappedTrans.frame_id_;
  odomAftMappedPtr->child_frame_id = _aftMappedTrans.child_frame_id_;
  odomAftMappedPtr->header.stamp = _timeLaserOdometry;
  odomAftMappedPtr->pose.pose.orientation.x = -geoQuat.y;
  odomAftMappedPtr->pose.pose.orientation.y = -geoQuat.z;
  odomAftMappedPtr->pose.pose.orientation.z = geoQuat.x;
  odomAftMappedPtr->pose.pose.orientation.w = geoQuat.w;
  odomAftMappedPtr->pose.pose.position.x = transformAftMapped().pos.x();
  odomAftMappedPtr->pose.pose.position.y = transformAftMapped().pos.y();
  odomAftMappedPtr->pose.pose.position.z = transformAftMapped().pos.z();
  odomAftMappedPtr->twist.twist.angular.x = transformBefMapped().rot_x.rad();
  odomAftMappedPtr->twist.twist.angular.y = transformBefMapped().rot_y.rad();
  odomAftMappedPtr->twist.twist.angular.z = transformBefMapped().rot_z.rad();
  odomAftMappedPtr->twist.twist.linear.x = transformBefMapped().pos.x();
  odomAftMappedPtr->twist.twist.linear.y = transformBefMapped().pos.y();
  odomAftMappedPtr->twist.twist.linear.z = transformBefMapped().pos.z();
  _pubOdomAftMapped.publish(odomAftMappedPtr);

  _aftMappedTrans.stamp_ = _timeLaserOdometry;
  _aftMappedTrans.setRotation(tf::Quaternion(-geoQuat.y, -geoQuat.z, geoQuat.x, geoQuat.w));
  _aftMappedTrans.setOrigin(tf::Vector3(transformAftMapped().pos.x(),
                                        transformAftMapped().pos.y(),
                                        transformAftMapped().pos.z()));
  _tfBroadcaster.sendTransform(_aftMappedTrans);

  // ARL Coordinate Frame Facilitators
  _gm_aftMappedTransformStamped_CORRECTED.header.stamp = _timeLaserOdometry;
  _gm_aftMappedTransformStamped_CORRECTED.transform.rotation.x = geoQuat.x;
  _gm_aftMappedTransformStamped_CORRECTED.transform.rotation.y = -geoQuat.y;
  _gm_aftMappedTransformStamped_CORRECTED.transform.rotation.z = -geoQuat.z;
  _gm_aftMappedTransformStamped_CORRECTED.transform.rotation.w = geoQuat.w;
  _gm_aftMappedTransformStamped_CORRECTED.transform.translation.x = transformAftMapped().pos.z();
  _gm_aftMappedTransformStamped_CORRECTED.transform.translation.y = transformAftMapped().pos.x();
  _gm_aftMappedTransformStamped_CORRECTED.transform.translation.z = transformAftMapped().pos.y();

  _pubAftMappedTransformStamped_CORRECTED.publish(_gm_aftMappedTransformStamped_CORRECTED);
  _tfBroadcaster.sendTransform(_gm_aftMappedTransformStamped_CORRECTED);

  nav_msgs::OdometryPtr _odomAftMapped_CORRECTEDPtr(new nav_msgs::Odometry);
  _odomAftMapped_CORRECTEDPtr->header.frame_id = _gm_aftMappedTransformStamped_CORRECTED.header.frame_id;
  _odomAftMapped_CORRECTEDPtr->child_frame_id = _gm_aftMappedTransformStamped_CORRECTED.child_frame_id;
  _odomAftMapped_CORRECTEDPtr->header.stamp = _timeLaserOdometry;
  _odomAftMapped_CORRECTEDPtr->pose.pose.orientation.x = geoQuat.x;
  _odomAftMapped_CORRECTEDPtr->pose.pose.orientation.y = -geoQuat.y;
  _odomAftMapped_CORRECTEDPtr->pose.pose.orientation.z = -geoQuat.z;
  _odomAftMapped_CORRECTEDPtr->pose.pose.orientation.w = geoQuat.w;
  _odomAftMapped_CORRECTEDPtr->pose.pose.position.x = transformAftMapped().pos.z();
  _odomAftMapped_CORRECTEDPtr->pose.pose.position.y = transformAftMapped().pos.x();
  _odomAftMapped_CORRECTEDPtr->pose.pose.position.z = transformAftMapped().pos.y();
  _odomAftMapped_CORRECTEDPtr->twist.twist.angular.x = transformBefMapped().rot_z.rad();
  _odomAftMapped_CORRECTEDPtr->twist.twist.angular.y = transformBefMapped().rot_x.rad();
  _odomAftMapped_CORRECTEDPtr->twist.twist.angular.z = transformBefMapped().rot_y.rad();
  _odomAftMapped_CORRECTEDPtr->twist.twist.linear.x = transformBefMapped().pos.z();
  _odomAftMapped_CORRECTEDPtr->twist.twist.linear.y = transformBefMapped().pos.x();
  _odomAftMapped_CORRECTEDPtr->twist.twist.linear.z = transformBefMapped().pos.y();
  _pubOdomAftMapped_CORRECTED.publish(_odomAftMapped_CORRECTEDPtr);
  // END ARL Coordinate Frame Facilitators

  //CUSTOMIZATION
  // Publish Path
  if (_pubPath.getNumSubscribers() > 0 || _forcePathPublishing) {
    //Pose Message
    geometry_msgs::PoseStamped poseMsg;
    poseMsg.header.frame_id = _gm_aftMappedTransformStamped_CORRECTED.header.frame_id;
    poseMsg.header.stamp = _timeLaserOdometry;
    poseMsg.pose.position.x = transformAftMapped().pos.z();
    poseMsg.pose.position.y = transformAftMapped().pos.x();
    poseMsg.pose.position.z = transformAftMapped().pos.y();
    poseMsg.pose.orientation.x = geoQuat.x;
    poseMsg.pose.orientation.y = -geoQuat.y;
    poseMsg.pose.orientation.z = -geoQuat.z;
    poseMsg.pose.orientation.w = geoQuat.w;

    // Path message
    _pathMsg.header.stamp = _timeLaserOdometry;
    _pathMsg.poses.push_back(poseMsg);

    //Publish
    _pubPath.publish(_pathMsg);
  }
  // Publish LaserMapping Optimization Status
  if (_pubOptimizationStatus.getNumSubscribers() > 0) {
    _optStatusMsg.header.stamp = _timeLaserOdometry;
    _pubOptimizationStatus.publish(_optStatusMsg);
  }

  //Publish full downsampled map for visualitzation purposes
  if (_pubMap.getNumSubscribers() > 0 && _mapCounter > _mapPublishRateDivider) {
    _mapCounter = 0;
    publishCloudMsg(_pubMap, laserMap(), _timeLaserOdometry, _aftMappedTrans.frame_id_);
  } else
    ++_mapCounter;

  // publish new submap cloud
  if (_pubSubmap.getNumSubscribers() > 0)
      publishCloudMsg(_pubSubmap, laserSubmap(), _timeLaserOdometry, _aftMappedTrans.frame_id_);

  //Publish submap search cloud
  if(_pubSubMapSearchCloud.getNumSubscribers() > 0)
    publishCloudMsg(_pubSubMapSearchCloud, getSubmapSearchCloud(), _timeLaserOdometry, _aftMappedTrans.frame_id_);
  //CUSTOMIZATION
}

}  // end namespace loam
