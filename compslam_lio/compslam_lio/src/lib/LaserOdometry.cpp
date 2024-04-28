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

#include "compslam_lio/LaserOdometry.h"

#include <pcl/filters/filter.h>

#include "compslam_lio/common.h"
#include "compslam_lio/read_ros_params.h"


namespace loam {

    using std::asin;
    using std::atan2;
    using std::cos;
    using std::fabs;
    using std::pow;
    using std::sin;
    using std::sqrt;

    LaserOdometry::LaserOdometry(float scanPeriod, uint16_t ioRatio, size_t maxIterations)
            : BasicLaserOdometry(scanPeriod, maxIterations),
              _ioRatio(ioRatio) {}

    bool LaserOdometry::setup(ros::NodeHandle &node, ros::NodeHandle &privateNode) {

        // Scan Period
        setScanPeriod(tryGetParam<float>("scanPeriod", privateNode));
        if (tryGetParam<float>("scanPeriod", privateNode) <= 0) {
            ROS_ERROR("Invalid scanPeriod parameter: (expected > 0)");
            return false;
        }

        // I/O ratio
        _ioRatio = tryGetParam<int>("ioRatio", privateNode);
        if (_ioRatio < 1) {
            ROS_ERROR("Invalid ioRatio parameter: (expected > 0)");
            return false;
        }

        // Odom Max Iterations
        setMaxIterations(tryGetParam<int>("odomMaxIterations", privateNode));
        if (tryGetParam<int>("odomMaxIterations", privateNode) < 1) {
            ROS_ERROR("Invalid odomMaxIterations parameter: (expected > 0)");
            return false;
        }

        // Odom Min Iterations
        setMinIterations(tryGetParam<int>("odomMinIterations", privateNode));
        if (tryGetParam<int>("odomMinIterations", privateNode) < 1) {
            ROS_ERROR("Invalid odomMinIterations parameter: (expected > 0)");
            return false;
        }

        // Delta T Abort
        setDeltaTAbort(tryGetParam<double>("odomDeltaTAbort", privateNode));
        if (tryGetParam<double>("odomDeltaTAbort", privateNode) <= 0) {
            ROS_ERROR("Invalid odomDeltaTAbort parameter: (expected > 0)");
            return false;
        }

        // Delta R Abort
        setDeltaRAbort(tryGetParam<double>("odomDeltaRAbort", privateNode));
        if (tryGetParam<double>("odomDeltaRAbort", privateNode) <= 0) {
            ROS_ERROR("Invalid odomDeltaRAbort parameter: (expected > 0)");
            return false;
        }

        //CUSTOMIZATION
        // Odometry Optimization Degenerate EigenValue
        setDegenEigenThreshold(tryGetParam<double>("odomDegenEigTh", privateNode));
        if (tryGetParam<double>("odomDegenEigTh", privateNode) <= 0) {
            ROS_ERROR("Invalid odomDegenEigVal parameter: (expected > 0)");
            return false;
        }

        //Odometry Translation Prior Max
        setExtOdomRelativeTranslationMax(tryGetParam<double>("odomRelativeTranslationMax", privateNode));
        if (tryGetParam<double>("odomRelativeTranslationMax", privateNode) <= 0) {
            ROS_ERROR("Invalid odomRelativeTranslationMax parameter: (expected > 0)");
            return false;
        }

        //Odometry Rotation Prior Max
        setExtOdomRelativeRotationMax(tryGetParam<double>("odomRelativeRotationMax", privateNode));
        if (tryGetParam<double>("odomRelativeRotationMax", privateNode) <= 0) {
            ROS_ERROR("Invalid odomRelativeRotationMax parameter: (expected > 0)");
            return false;
        }

        //External Prior
        setExternalPriorStatus(tryGetParam<bool>("extPriorAvailable", privateNode));
        //Set external ODOMETRY frame
        setExternalOdometryFrame(tryGetParam<std::string>("extOdomFrame", privateNode));

        //Set external FIXED frame
        setExternalFixedFrame(tryGetParam<std::string>("extFixedFrame", privateNode));

        //Set external SENSOR frame
        setExternalSensorFrame(tryGetParam<std::string>("extSensorFrame", privateNode));

        //Set external odometry time offset
        setExternalOdometryTimeOffset(tryGetParam<double>("extOdomTimeOffset", privateNode));

        //Fallback External Prior
        setFallbackExternalPriorStatus(tryGetParam<bool>("fallbackExtPriorAvailable", privateNode));
        //Set external ODOMETRY frame
        setFallbackExternalOdometryFrame(tryGetParam<std::string>("fallbackExtOdomFrame", privateNode));

        //Set external FIXED frame
        setFallbackExternalFixedFrame(tryGetParam<std::string>("fallbackExtFixedFrame", privateNode));

        //Set external SENSOR frame
        setFallbackExternalSensorFrame(tryGetParam<std::string>("fallbackExtSensorFrame", privateNode));

        //Set fallback external odometry time offset
        setFallbackExternalOdomTimeOffset(tryGetParam<double>("fallbackExtOdomTimeOffset", privateNode));

        //Set LiDAR frame
        setLidarFrame(tryGetParam<std::string>("lidarFrame", privateNode));

        //Set input PCL rotation w.r.t LiDAR frame
        setInputPCLRotationStatus(tryGetParam<bool>("rotateInputCloud", privateNode));
        std::vector<double> ypr{0.0, 0.0, 0.0};
        ypr = tryGetParam<std::vector<double>>("inputCloudRotation", privateNode);
        if (ypr.size() == 3) {
            Eigen::Quaterniond q = Eigen::AngleAxisd(ypr[0], Eigen::Vector3d::UnitZ()) *
                                   Eigen::AngleAxisd(ypr[1], Eigen::Vector3d::UnitY()) *
                                   Eigen::AngleAxisd(ypr[2], Eigen::Vector3d::UnitX());
            q.normalize();
            tf::Transform T(tf::Quaternion(q.x(), q.y(), q.z(), q.w()),
                            tf::Vector3(0, 0, 0)); /*quaternion order: x,y,z,w*/
            setLiDARtoInputPCLRotation(T);
            ROS_INFO_STREAM(
                    "LaserOdometry - Rotation applied to input clound in order YPR(rad): [" << ypr[0] << " "
                                                                                            << ypr[1] << " "
                                                                                            << ypr[2]
                                                                                            << "]");
        } else {
            ROS_WARN("LaserOdometry - Input cloud rotation not set - DISABLING INPUT CLOUD ROTATION");
            setInputPCLRotationStatus(false);
            throw std::runtime_error("LaserOdometry - Input cloud rotation not set - DISABLING INPUT CLOUD ROTATION");
        }

        //Set input cloud undistortion status - if distorted input then approx ego motion compensation will be performed using external prior(if available) or motion model
        setUndistortInputCloud(tryGetParam<bool>("undistortInputCloud", privateNode)
        );

        /** Graph Parameters **/
        //IMU Preintegrtor
        setImuFrame(tryGetParam<std::string>("imuFrame", privateNode)
        );
        setImuTimeOffset(tryGetParam<double>("imuTimeOffset", privateNode)
        );
        _imuBuffer.
                setImuRate(tryGetParam<double>("imuRate", privateNode)
        );
        _imuBuffer.
                setImuTimeForInit(tryGetParam<double>("imuTimeForInit", privateNode)
        );
        _graphMgr.
                setGlobalZaxisPointsUp(tryGetParam<bool>("globalZaxisPointsUp", privateNode)
        );
        _graphMgr.
                setAccNoiseDensity(tryGetParam<double>("accNoiseDensity", privateNode)
        );
        _graphMgr.
                setAccBiasRandomWalk(tryGetParam<double>("accBiasRandomWalk", privateNode)
        );
        _graphMgr.
                setGyroNoiseDensity(tryGetParam<double>("gyrNoiseDensity", privateNode)
        );
        _graphMgr.
                setGyrBiasRandomWalk(tryGetParam<double>("gyrBiasRandomWalk", privateNode)
        );
        _graphMgr.
                setImuIntegrationCovariance(tryGetParam<double>("imuIntegrationCovariance", privateNode)
        );
        _graphMgr.
                setImuBiasAccOmegaInt(tryGetParam<double>("imuBiasAccOmegaInt", privateNode)
        );
        _graphMgr.
                setAccBiasPrior(tryGetParam<std::vector<double>>("accBiasPrior", privateNode)
        );
        _graphMgr.
                setGyrBiasPrior(tryGetParam<std::vector<double>>("gyrBiasPrior", privateNode)
        );
        //Factor Graph
        _graphMgr.
                setSmootherLag(tryGetParam<double>("smootherLag", privateNode)
        );
        _graphMgr.
                setIterations(tryGetParam<int>("additonalIterations", privateNode)
        );
        _graphMgr.
                setPositionReLinTh(tryGetParam<double>("positionReLinTh", privateNode)
        );
        _graphMgr.
                setRotationReLinTh(tryGetParam<double>("rotationReLinTh", privateNode)
        );
        _graphMgr.
                setVelocityReLinTh(tryGetParam<double>("velocityReLinTh", privateNode)
        );
        _graphMgr.
                setAccBiasReLinTh(tryGetParam<double>("accBiasReLinTh", privateNode)
        );
        _graphMgr.
                setGyrBiasReLinTh(tryGetParam<double>("gyrBiasReLinTh", privateNode)
        );
        _graphMgr._params.
                relinearizeSkip = tryGetParam<int>("relinearizeSkip", privateNode);
        _graphMgr._params.
                enableRelinearization = tryGetParam<bool>("enableRelinearization", privateNode);
        _graphMgr._params.
                evaluateNonlinearError = tryGetParam<bool>("evaluateNonlinearError", privateNode);
        _graphMgr._params.
                cacheLinearizedFactors = tryGetParam<bool>("cacheLinearizedFactors", privateNode);
        _graphMgr._params.
                findUnusedFactorSlots = tryGetParam<bool>("findUnusedFactorSlots", privateNode);
        _graphMgr._params.
                enablePartialRelinearizationCheck = tryGetParam<bool>("enablePartialRelinearizationCheck",
                                                                      privateNode);
        _graphMgr._params.
                enableDetailedResults = tryGetParam<bool>("enableDetailedResults", privateNode);
        //Pose between factor noise
        _graphMgr.
                setPoseNoise(tryGetParam<std::vector<double>>("poseBetweenNoise", privateNode)
        );
        //Zero Motion Factor & Gravity Roll/Pitch Factor
        setZeroMotionDetection(tryGetParam<bool>("zeroMotionDetection", privateNode)
        );
        _graphMgr.
                setZeroMotionTh(tryGetParam<double>("zeroMotionThreshold", privateNode)
        );
        //Minimum consective zero motion detection
        _graphMgr.
                setMinZeroMotionDetections(tryGetParam<int>("minZeroMoitionDetections", privateNode)
        );
        //Check if Gravity-algined Roll-Pitch factor should be added
        setUseGravityRollPitchFactors(tryGetParam<bool>("gravityRollPitchFactors", privateNode)
        );
        // Initialize graph roll/pitch from IMU
        setUseIMURollPitchForGraphInit(tryGetParam<bool>("initGraphRollPitchFromIMU", privateNode)
        );
        if (!tryGetParam<bool>("initGraphRollPitchFromIMU", privateNode)) {
            setUseGravityRollPitchFactors(false);
            ROS_INFO_STREAM(
                    "LaserOdometry - Zero Motion Detection DISABLED as Graph Roll/Pitch NOT initialized from IMU");
        }
/** Graph Parameters **/

//Verbose Ouput
        setVerboseLevel(tryGetParam<int>("compslamLioVerbosity", privateNode)
        );
//CUSTOMIZATION

//Get node namespace for tf and remove leading "/" and add lagging "/" if namespace is not empty
        std::string tf_ns = node.getNamespace();
        if (!tf_ns.

                empty()

                )
            tf_ns = tf_ns.erase(0, 1) + "/";

// initialize tf messages
        _laserOdometryTrans.
                frame_id_ = tf_ns + "camera_init";
        _laserOdometryTrans.
                child_frame_id_ = tf_ns + "laser_odom";
        _cameraFrameId = "camera";
        _laserOdometryTrans_CORRECTED.
                frame_id_ = tf_ns + "camera_init_CORRECTED";
        _laserOdometryTrans_CORRECTED.
                child_frame_id_ = tf_ns + "laser_odom_CORRECTED";

// initialize gravity aligned tf message
        _cameraInitGravityAlignedTf.
                frame_id_ = _laserOdometryTrans_CORRECTED.frame_id_;
        _cameraInitGravityAlignedTf.
                child_frame_id_ = tf_ns + "camera_init_GRAVITY_ALIGNED";
        _cameraInitGravityAlignedTf.
                setOrigin(tf::Vector3(0.0, 0.0, 0.0)
        );

// advertise laser odometry topics
        _pubLaserCloudCornerLast = node.advertise<sensor_msgs::PointCloud2>("laser_cloud_corner_last", 2);
        _pubLaserCloudSurfLast = node.advertise<sensor_msgs::PointCloud2>("laser_cloud_surf_last", 2);
        _pubLaserCloudFullRes = node.advertise<sensor_msgs::PointCloud2>("velodyne_cloud_3", 2);
        _pubLaserOdometry = node.advertise<nav_msgs::Odometry>("laser_odom_to_init", 5);

// subscribe input Cloud topics using ROS exact sync policy in a single callback
        _subSharp.
                subscribe(node,
                          "laser_cloud_sharp", 2);
        _subLessSharp.
                subscribe(node,
                          "laser_cloud_less_sharp", 2);
        _subFlat.
                subscribe(node,
                          "laser_cloud_flat", 2);
        _subLessFlat.
                subscribe(node,
                          "laser_cloud_less_flat", 2);
        _subFull.
                subscribe(node,
                          "laser_cloud_all", 2);
        _pclExactSyncPtr.reset(
                new
                        message_filters::Synchronizer<_pclExactSyncPolicy>(_pclExactSyncPolicy(10), _subSharp,
                                                                           _subLessSharp, _subFlat, _subLessFlat,
                                                                           _subFull
                ));
        _pclExactSyncPtr->
                registerCallback(boost::bind(&LaserOdometry::syncCloudsHandler, this, _1, _2, _3, _4, _5)
        );

// subscribe to IMU topic
        _subImuTrans = node.subscribe<sensor_msgs::PointCloud2>("imu_trans", 5, &LaserOdometry::imuTransHandler, this);
        _subImu = node.subscribe<sensor_msgs::Imu>("imu", 400, &LaserOdometry::imuCallback, this,
                                                   ros::TransportHints().tcpNoDelay());

// ARL Coordinate Frame Facilitators
        _pubLaserOdometry_CORRECTED = node.advertise<nav_msgs::Odometry>("laser_odom_to_init_CORRECTED", 5);
// END ARL Coordinate Frame Facilitators

//CUSTOMIZATION
        _pubOptimizationStatus = node.advertise<compslam_lio_msgs::OptStatus>("laser_odom_optimization_status", 5);
        _pubLaserOdometryStatus = node.advertise<std_msgs::Header>("laser_odom_status", 5);
        _pubLaserImuBias = node.advertise<sensor_msgs::Imu>("laser_odom_imu_bias", 20);
//CUSTOMIZATION
        return true;
    }  // namespace loam

    void LaserOdometry::laserCloudSharpHandler(const sensor_msgs::PointCloud2ConstPtr &cornerPointsSharpMsg) {
        _timeCornerPointsSharp = cornerPointsSharpMsg->header.stamp;

        cornerPointsSharp()->clear();
        pcl::fromROSMsg(*cornerPointsSharpMsg, *cornerPointsSharp());
        std::vector<int> indices;
        pcl::removeNaNFromPointCloud(*cornerPointsSharp(), *cornerPointsSharp(), indices);
    }

    void LaserOdometry::laserCloudLessSharpHandler(const sensor_msgs::PointCloud2ConstPtr &cornerPointsLessSharpMsg) {
        _timeCornerPointsLessSharp = cornerPointsLessSharpMsg->header.stamp;

        cornerPointsLessSharp()->clear();
        pcl::fromROSMsg(*cornerPointsLessSharpMsg, *cornerPointsLessSharp());
        std::vector<int> indices;
        pcl::removeNaNFromPointCloud(*cornerPointsLessSharp(), *cornerPointsLessSharp(), indices);
    }

    void LaserOdometry::laserCloudFlatHandler(const sensor_msgs::PointCloud2ConstPtr &surfPointsFlatMsg) {
        _timeSurfPointsFlat = surfPointsFlatMsg->header.stamp;

        surfPointsFlat()->clear();
        pcl::fromROSMsg(*surfPointsFlatMsg, *surfPointsFlat());
        std::vector<int> indices;
        pcl::removeNaNFromPointCloud(*surfPointsFlat(), *surfPointsFlat(), indices);
    }

    void LaserOdometry::laserCloudLessFlatHandler(const sensor_msgs::PointCloud2ConstPtr &surfPointsLessFlatMsg) {
        _timeSurfPointsLessFlat = surfPointsLessFlatMsg->header.stamp;

        surfPointsLessFlat()->clear();
        pcl::fromROSMsg(*surfPointsLessFlatMsg, *surfPointsLessFlat());
        std::vector<int> indices;
        pcl::removeNaNFromPointCloud(*surfPointsLessFlat(), *surfPointsLessFlat(), indices);
    }

    void LaserOdometry::laserCloudFullResHandler(const sensor_msgs::PointCloud2ConstPtr &laserCloudFullResMsg) {
        _timeLaserCloudFullRes = laserCloudFullResMsg->header.stamp;

        laserCloud()->clear();
        pcl::fromROSMsg(*laserCloudFullResMsg, *laserCloud());
        std::vector<int> indices;
        pcl::removeNaNFromPointCloud(*laserCloud(), *laserCloud(), indices);
    }

    void LaserOdometry::imuTransHandler(const sensor_msgs::PointCloud2ConstPtr &imuTransMsg) {
        _timeImuTrans = imuTransMsg->header.stamp;

        pcl::PointCloud<pcl::PointXYZ> imuTrans;
        pcl::fromROSMsg(*imuTransMsg, imuTrans);
        updateIMU(imuTrans);
    }

    void LaserOdometry::imuCallback(const sensor_msgs::Imu::ConstPtr &imu_ptr) {
        //Add to buffer
        _imuBuffer.addToIMUBuffer(imu_ptr->header.stamp.toSec(),
                                  imu_ptr->linear_acceleration.x, imu_ptr->linear_acceleration.y,
                                  imu_ptr->linear_acceleration.z,
                                  imu_ptr->angular_velocity.x, imu_ptr->angular_velocity.y,
                                  imu_ptr->angular_velocity.z);
    }

    void LaserOdometry::syncCloudsHandler(const sensor_msgs::PointCloud2ConstPtr &pclSharp,
                                          const sensor_msgs::PointCloud2ConstPtr &pclLessSharp,
                                          const sensor_msgs::PointCloud2ConstPtr &pclFlat,
                                          const sensor_msgs::PointCloud2ConstPtr &pclLessFlat,
                                          const sensor_msgs::PointCloud2ConstPtr &pclFull) {
        //Process incoming pointclouds
        laserCloudSharpHandler(pclSharp);
        laserCloudLessSharpHandler(pclLessSharp);
        laserCloudFlatHandler(pclFlat);
        laserCloudLessFlatHandler(pclLessFlat);
        laserCloudFullResHandler(pclFull);

        // try processing new data
        process();
    }

    void LaserOdometry::process() {
        //Update Timestamp according to timestamp of original PCL
        updatePCLTime(_timeSurfPointsLessFlat);

        BasicLaserOdometry::process();
        publishResult();
    }

    void LaserOdometry::publishResult() {
        // publish odometry transformations
        geometry_msgs::Quaternion geoQuat = tf::createQuaternionMsgFromRollPitchYaw(transformSum().rot_z.rad(),
                                                                                    -transformSum().rot_x.rad(),
                                                                                    -transformSum().rot_y.rad());

        // initialize odometry and odometry tf messages
        nav_msgs::OdometryPtr laserOdometryMsgPtr(new nav_msgs::Odometry);
        laserOdometryMsgPtr->header.frame_id = _laserOdometryTrans.frame_id_;
        laserOdometryMsgPtr->child_frame_id = _laserOdometryTrans.child_frame_id_;
        laserOdometryMsgPtr->header.stamp = _timeSurfPointsLessFlat;
        laserOdometryMsgPtr->pose.pose.orientation.x = -geoQuat.y;
        laserOdometryMsgPtr->pose.pose.orientation.y = -geoQuat.z;
        laserOdometryMsgPtr->pose.pose.orientation.z = geoQuat.x;
        laserOdometryMsgPtr->pose.pose.orientation.w = geoQuat.w;
        laserOdometryMsgPtr->pose.pose.position.x = transformSum().pos.x();
        laserOdometryMsgPtr->pose.pose.position.y = transformSum().pos.y();
        laserOdometryMsgPtr->pose.pose.position.z = transformSum().pos.z();
        _pubLaserOdometry.publish(laserOdometryMsgPtr);

        _laserOdometryTrans.stamp_ = _timeSurfPointsLessFlat;
        _laserOdometryTrans.setRotation(tf::Quaternion(-geoQuat.y, -geoQuat.z, geoQuat.x, geoQuat.w));
        _laserOdometryTrans.setOrigin(
                tf::Vector3(transformSum().pos.x(), transformSum().pos.y(), transformSum().pos.z()));
        _tfBroadcaster.sendTransform(_laserOdometryTrans);

        // ARL Coordinate Frame Facilitators
        nav_msgs::OdometryPtr laserOdometry_CORRECTEDPtr(new nav_msgs::Odometry);
        laserOdometry_CORRECTEDPtr->header.frame_id = _laserOdometryTrans_CORRECTED.frame_id_;
        laserOdometry_CORRECTEDPtr->child_frame_id = _laserOdometryTrans_CORRECTED.child_frame_id_;
        laserOdometry_CORRECTEDPtr->header.stamp = _timeSurfPointsLessFlat;
        laserOdometry_CORRECTEDPtr->pose.pose.orientation.x = geoQuat.x;
        laserOdometry_CORRECTEDPtr->pose.pose.orientation.y = -geoQuat.y;
        laserOdometry_CORRECTEDPtr->pose.pose.orientation.z = -geoQuat.z;
        laserOdometry_CORRECTEDPtr->pose.pose.orientation.w = geoQuat.w;
        laserOdometry_CORRECTEDPtr->pose.pose.position.x = transformSum().pos.z();  //smk: LOAM doesn't follow camera convention but Z-Forward, X-Left and Y-Up
        laserOdometry_CORRECTEDPtr->pose.pose.position.y = transformSum().pos.x();
        laserOdometry_CORRECTEDPtr->pose.pose.position.z = transformSum().pos.y();
        _pubLaserOdometry_CORRECTED.publish(laserOdometry_CORRECTEDPtr);

        _laserOdometryTrans_CORRECTED.stamp_ = _timeSurfPointsLessFlat;
        _laserOdometryTrans_CORRECTED.setRotation(tf::Quaternion(laserOdometry_CORRECTEDPtr->pose.pose.orientation.x,
                                                                 laserOdometry_CORRECTEDPtr->pose.pose.orientation.y,
                                                                 laserOdometry_CORRECTEDPtr->pose.pose.orientation.z,
                                                                 laserOdometry_CORRECTEDPtr->pose.pose.orientation.w));
        _laserOdometryTrans_CORRECTED.setOrigin(tf::Vector3(laserOdometry_CORRECTEDPtr->pose.pose.position.x,
                                                            laserOdometry_CORRECTEDPtr->pose.pose.position.y,
                                                            laserOdometry_CORRECTEDPtr->pose.pose.position.z));
        _tfBroadcaster.sendTransform(_laserOdometryTrans_CORRECTED);
        // END ARL Coordinate Frame Facilitators

        //IMU Bias
        if (_pubLaserImuBias.getNumSubscribers() > 0) {
            sensor_msgs::Imu imuBiasMsg;
            imuBiasMsg.header.frame_id = "/laser_odometry_imu";
            imuBiasMsg.header.stamp = _timeSurfPointsLessFlat;
            imuBiasMsg.linear_acceleration.x = graphIMUBias().linear_acceleration.x;
            imuBiasMsg.linear_acceleration.y = graphIMUBias().linear_acceleration.y;
            imuBiasMsg.linear_acceleration.z = graphIMUBias().linear_acceleration.z;
            imuBiasMsg.angular_velocity.x = graphIMUBias().angular_velocity.x;
            imuBiasMsg.angular_velocity.y = graphIMUBias().angular_velocity.y;
            imuBiasMsg.angular_velocity.z = graphIMUBias().angular_velocity.z;
            _pubLaserImuBias.publish(imuBiasMsg);
        }
        //CUSTOMIZATION
        //Publish LaserOdometry Optimization Status
        if (_pubOptimizationStatus.getNumSubscribers() > 0) {
            _optStatusMsg.header.stamp = _timeSurfPointsLessFlat;
            _pubOptimizationStatus.publish(_optStatusMsg);
        }
        //Publish LaserOdometry Status
        std_msgs::HeaderPtr laserOdomStatusMsgPtr(new std_msgs::Header);
        laserOdomStatusMsgPtr->stamp = _timeSurfPointsLessFlat;
        laserOdomStatusMsgPtr->seq = (getLaserOdometryStatus() ? 1 : 0);
        _pubLaserOdometryStatus.publish(laserOdomStatusMsgPtr);
        //Publish Gravity Aligned Init frame
        if (gravityAttitudeInitialized()) {
            _cameraInitGravityAlignedTf.stamp_ = _timeSurfPointsLessFlat;
            _cameraInitGravityAlignedTf.setBasis(getLiDARGravityInitAttitude().getBasis());
            _tfBroadcaster.sendTransform(_cameraInitGravityAlignedTf);
        }
        //CUSTOMIZATION

        // publish cloud results according to the input output ratio
        if (_ioRatio < 2 || frameCount() % _ioRatio == 1) {
            publishCloudMsg(_pubLaserCloudCornerLast, *lastCornerCloud(), _timeSurfPointsLessFlat,
                            _cameraFrameId);  //smk: becuase of _ioRatio=2 this is 5Hz
            publishCloudMsg(_pubLaserCloudSurfLast, *lastSurfaceCloud(), _timeSurfPointsLessFlat, _cameraFrameId);
            transformToEnd(
                    laserCloud());  // transform full resolution cloud to sweep end before sending it ///smk: velodyne_cloud_2 is transformed and sent out as /velodyne_cloud_3
            publishCloudMsg(_pubLaserCloudFullRes, *laserCloud(), _timeSurfPointsLessFlat, _cameraFrameId);
        }
    }

}  // end namespace loam
