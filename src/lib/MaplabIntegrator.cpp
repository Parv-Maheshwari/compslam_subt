#include "loam/MaplabIntegrator.hpp"

namespace loam {

bool MaplabIntegrator::setup(ros::NodeHandle& node, ros::NodeHandle& privateNode) {
  //Parameters
  //Frames
  if (privateNode.getParam("world_frame", _world_frame))
    ROS_INFO_STREAM("MaplabIntegrator - World/Global frame set to: " << _world_frame);
  if (privateNode.getParam("map_frame", _map_frame))
    ROS_INFO_STREAM("MaplabIntegrator - Robot Map frame set to: " << _map_frame);
  if (privateNode.getParam("base_frame", _base_frame))
    ROS_INFO_STREAM("MaplabIntegrator - Robot Base frame set to: " << _base_frame);
  if (privateNode.getParam("publishWorldtoMapTransform", _publishWorldtoMapTf))
    if (_publishWorldtoMapTf) ROS_WARN_STREAM("MaplabIntegrator - WORLD(" << _world_frame << ")-to-MAP(" << _map_frame << ") Transform Pubslihed --- typically used for debugging.");
  //Pointcloud save delta
  if (privateNode.getParam("cloudSavePosDelta", _cloudSavePosDelta))
    ROS_INFO_STREAM("MaplabIntegrator - Minimum Position(m) delta Norm for saving pointcloud: " << _cloudSavePosDelta);
  if (privateNode.getParam("cloudSaveRotDelta", _cloudSaveRotDelta))
    ROS_INFO_STREAM("MaplabIntegrator - Minimum Rotation(rad) delta Norm for saving pointcloud: " << _cloudSaveRotDelta);
  //Extrinsic Calibrations
  std::vector<double> T_cam2imu;  //Camera to IMU
  if (privateNode.getParam("T_cam2imu", T_cam2imu)) {
    _T_B_C = gtsam::Pose3(Eigen::Map<Eigen::Matrix<double, 4, 4, Eigen::RowMajor>>(T_cam2imu.data()));
    std::cout << "MaplabIntegrator - T_cam2imu:\n"
              << _T_B_C.matrix() << std::endl;
  } else {
    ROS_INFO_STREAM("MaplabIntegrator - T_cam2imu not defined");
    return false;
  }
  std::vector<double> T_imu2lidar;  //IMU to Lidar
  if (privateNode.getParam("T_imu2lidar", T_imu2lidar)) {
    _T_L_B = gtsam::Pose3(Eigen::Map<Eigen::Matrix<double, 4, 4, Eigen::RowMajor>>(T_imu2lidar.data()));
    std::cout << "MaplabIntegrator - T_imu2lidar:\n"
              << _T_L_B.matrix() << std::endl;
  } else {
    ROS_INFO_STREAM("MaplabIntegrator - T_imu2lidar not defined");
    return false;
  }
  //Factor Noise parameters
  std::vector<double> odomNoise;
  if (privateNode.getParam("odometryFactorNoise", odomNoise)) {
    _odomNoise << odomNoise[0], odomNoise[1], odomNoise[2], odomNoise[3], odomNoise[4], odomNoise[5];  // rad,rad,rad,m,m,m
    std::cout << "MaplabIntegrator - Odometry Factor Noise: " << _odomNoise.transpose() << std::endl;
  } else {
    ROS_INFO_STREAM("MaplabIntegrator - Odometry Factor Noise not defined");
    return false;
  }
  std::vector<double> absNoise;
  if (privateNode.getParam("absoluteFactorNoise", absNoise)) {
    _absoluteNoise << absNoise[0], absNoise[1], absNoise[2], absNoise[3], absNoise[4], absNoise[5];  // rad,rad,rad,m,m,m
    std::cout << "MaplabIntegrator - Absolute Factor Noise: " << _absoluteNoise.transpose() << std::endl;
  } else {
    ROS_INFO_STREAM("MaplabIntegrator - Absolute Factor Noise not defined");
    return false;
  }
  std::vector<double> submapNoise;
  if (privateNode.getParam("submapFactorNoise", submapNoise)) {
    _submapNoise << submapNoise[0], submapNoise[1], submapNoise[2], submapNoise[3], submapNoise[4], submapNoise[5];  // rad,rad,rad,m,m,m
    std::cout << "MaplabIntegrator - Submap Factor Noise: " << _submapNoise.transpose() << std::endl;
  } else {
    ROS_INFO_STREAM("MaplabIntegrator - Submap Factor Noise not defined");
    return false;
  }
  std::vector<double> anchorNoise;
  if (privateNode.getParam("anchorFactorNoise", anchorNoise)) {
    _anchorNoise << anchorNoise[0], anchorNoise[1], anchorNoise[2], anchorNoise[3], anchorNoise[4], anchorNoise[5];  // rad,rad,rad,m,m,m
    std::cout << "MaplabIntegrator - Anchor Factor Noise: " << _anchorNoise.transpose() << std::endl;
  } else {
    ROS_INFO_STREAM("MaplabIntegrator - Anchor Factor Noise not defined");
    return false;
  }
  //Full graph result update interval
  privateNode.getParam("updateResultsInterval", _updateResultsInterval);
  ROS_INFO_STREAM("MaplabIntegrator - Full Graph Result Updated Every: " << _updateResultsInterval << " seconds");

  //Synced subscriber
  _subOdom.subscribe(node, "odometry", 10);
  _subCloud.subscribe(node, "pointcloud", 10);
  _syncPtr.reset(new message_filters::Synchronizer<_syncPolicy>(_syncPolicy(100), _subOdom, _subCloud));
  _syncPtr->registerCallback(boost::bind(&MaplabIntegrator::syncCallbackHandler, this, _1, _2));
  //Subscribers
  _subAbsolutePose = node.subscribe<geometry_msgs::PoseWithCovarianceStamped>("absolute_constraints", 100, &MaplabIntegrator::absolutePoseCallback, this);
  _subMaplabAnchor = node.subscribe<nav_msgs::Path>("anchor_constraints", 100, &MaplabIntegrator::maplabAnchorCallback, this);
  _subMaplabSubmap = node.subscribe<nav_msgs::Path>("submap_constraints", 100, &MaplabIntegrator::maplabSubmapCallback, this);

  //Publisher
  _pubIncrementalPath = node.advertise<nav_msgs::Path>("/incremental_trajectory", 10);
  _pubUpdatedPath = node.advertise<nav_msgs::Path>("/optimized_path", 10);
  _pathMsg.poses.reserve(40000);  //1 hour path @ 10Hz ~ 36000
  _pubMap = node.advertise<sensor_msgs::PointCloud2>("/laser_map_optimized", 10);
  _pubIncPoseStamped = node.advertise<geometry_msgs::PoseStamped>("/incremental_pose", 10);
  _pubOptPoseStamped = node.advertise<geometry_msgs::PoseStamped>("/optimized_pose", 10);

  //Timer-callback for graph update
  _timerUpdateResults = node.createTimer(ros::Duration(_updateResultsInterval), std::bind(&MaplabIntegrator::updateGraphResults, this));

  //Set factor graph params
  _params.optimizationParams = gtsam::ISAM2GaussNewtonParams();
  _params.factorization = gtsam::ISAM2Params::QR;  //CHOLESKY:Fast but non-stable //QR:Slower but more stable in poorly conditioned problems
  gtsam::FastMap<char, gtsam::Vector> relinTh;     //Set graph relinearization thresholds - must be lower case letters, check:gtsam::symbol_shorthand
  relinTh['x'] = (gtsam::Vector(6) << 0.05, 0.05, 0.05, 0.05, 0.05, 0.05).finished();
  _params.relinearizeThreshold = relinTh;
  _params.setEnableRelinearization(true);
  _params.setRelinearizeSkip(1);
  _params.setCacheLinearizedFactors(false);
  _params.setEvaluateNonlinearError(false);
  _params.findUnusedFactorSlots = false;  //NOTE: Set false for factor removal lookup
  _params.setEnablePartialRelinearizationCheck(true);
  _params.setEnableDetailedResults(false);
  //Initialize factor graph object
  _graph = std::make_shared<gtsam::ISAM2>(_params);
  _graph->params().print("Graph Integrator - Parameters:");

  return true;
}

void MaplabIntegrator::syncCallbackHandler(const nav_msgs::OdometryConstPtr& OdomPtr, const sensor_msgs::PointCloud2ConstPtr& cloudPtr) {
  //Add odometry factor and check if pointcloud needs to be stored
  int key = lidarOdomCallback(OdomPtr);
  cloudCallback(key, cloudPtr);
}

int MaplabIntegrator::lidarOdomCallback(const nav_msgs::Odometry::ConstPtr& odomPtr) {
  if (odomPtr == nullptr) {
    ROS_WARN("MaplabIntegrator - nullptr passed to Odometry callback");
    return -1;
  }

  //Current timestamp and LiDAR pose
  const double ts = odomPtr->header.stamp.toSec();
  const auto& p = odomPtr->pose.pose;
  const gtsam::Pose3 T_M_L(gtsam::Rot3(p.orientation.w, p.orientation.x, p.orientation.y, p.orientation.z),
                           gtsam::Point3(p.position.x, p.position.y, p.position.z));
  //IMU pose
  gtsam::Pose3 T_M_B = T_M_L * _T_L_B;
  //Compute delta in IMU frame
  if (_firstOdomMsg)  //Store first pose to compute zero delta incase input doesn't start from zero
    _lastIMUPose = T_M_B;
  gtsam::Pose3 T_B1_B2 = _lastIMUPose.between(T_M_B);

  //Add delta pose factor to graph
  gtsam::Key new_key;
  bool saveCloud = false;
  auto t1 = std::chrono::high_resolution_clock::now();
  {
    //If first lidar odometry message has been received
    std::lock_guard<std::mutex> lock(_graphMutex);
    if (_firstOdomMsg) {
      //Initialize - Add temporary identity prior factor
      new_key = stateKey();
      addPriorFactor(new_key, gtsam::Pose3::identity());
      saveCloud = true;
      _firstOdomMsg = false;
    } else {
      //Update keys
      auto old_key = _timestampKeyMap.rbegin()->second;
      new_key = newStateKey();
      //Calculate new IMU pose estimate in G
      gtsam::Pose3 T_G_B = _T_G_B_inc * T_B1_B2;
      //Add delta and estimate as between factor
      addPoseBetweenFactor(old_key, new_key, T_B1_B2, T_G_B);
    }
    //Update Timestamp-Key Map
    _timestampKeyMap[ts] = new_key;
    //Get updated result
    _T_G_B_inc = _graph->calculateEstimate<gtsam::Pose3>(X(new_key));
  }
  auto t2 = std::chrono::high_resolution_clock::now();

  //Save last IMU pose for calculting delta
  _lastIMUPose = T_M_B;

  //Check motion difference to know if incoming cloud should be saved
  gtsam::Pose3 cloud_delta = _lastCloudSavePose.between(T_M_L);
  if (cloud_delta.translation().norm() > _cloudSavePosDelta ||
      cloud_delta.rotation().rpy().norm() > _cloudSaveRotDelta) {
    _lastCloudSavePose = T_M_L;
    saveCloud = true;
  }

  //Publish Graph path
  //Pose Message
  static size_t poseMsgCount = 0;
  geometry_msgs::PoseStamped poseMsg;
  poseMsg.header.frame_id = _world_frame;
  poseMsg.header.stamp = odomPtr->header.stamp;
  poseMsg.header.seq = ++poseMsgCount;
  createPoseMessage(_T_G_B_inc, &poseMsg);

  //Node Message
  _pathMsg.header.frame_id = _world_frame;
  _pathMsg.header.stamp = poseMsg.header.stamp;
  _pathMsg.poses.emplace_back(std::move(poseMsg));

  //Publish Path
  _pubIncrementalPath.publish(_pathMsg);

  //Publish Transforms
  publishTransforms(odomPtr->header.stamp);

  // DEBUG
  if (new_key % 10 == 0) {
    std::cout << std::fixed << "\033[35mODOMETRY\033[0m"
              << ", ts: " << odomPtr->header.stamp.toSec()
              << ", key: " << new_key
              // << ", Total factors: " << getFactorCount()
              << ", T_G_B t(m): " << _T_G_B_inc.translation().transpose()  //<< ", RPY(deg): " << _T_G_B_inc.rotation().rpy().transpose() * (180.0 / M_PI)
              << ", time(ms): " << std::chrono::duration_cast<std::chrono::milliseconds>(t2 - t1).count()
              << std::endl;
  }
  // DEBUG

  //return key to associate pointcloud
  if (saveCloud)
    return new_key;
  else
    return -1;
}

void MaplabIntegrator::cloudCallback(int key, const sensor_msgs::PointCloud2ConstPtr& cloudPtr) {
  if (cloudPtr == nullptr) {
    ROS_WARN("MaplabIntegrator - nullptr passed to Pointcloud callback");
    return;
  }

  //Check if pointcloud needs to be stored
  if (key < 0)
    return;

  //New pointcloud
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromROSMsg(*cloudPtr, *cloud);

  // Passthrough filter
  pcl::PassThrough<pcl::PointXYZ> pass;
  // Filter - X
  pass.setInputCloud(cloud);
  pass.setFilterFieldName("x");
  pass.setFilterLimits(-30, 30);
  pass.setFilterLimitsNegative(false);
  pass.filter(*cloud);
  // Filter - Y
  pass.setInputCloud(cloud);
  pass.setFilterFieldName("y");
  pass.setFilterLimits(-30, 30);
  pass.setFilterLimitsNegative(false);
  pass.filter(*cloud);
  // Filter - Z
  pass.setInputCloud(cloud);
  pass.setFilterFieldName("z");
  pass.setFilterLimits(-15, 15);
  pass.setFilterLimitsNegative(false);
  pass.filter(*cloud);

  // VoxelGrid Filter
  pcl::VoxelGrid<pcl::PointXYZ> voxGrid;
  voxGrid.setLeafSize(0.4, 0.4, 0.4);
  voxGrid.setInputCloud(cloud);
  voxGrid.filter(*cloud);

  //Add cloud
  _keyCloudMap[key] = cloud;
}

void MaplabIntegrator::absolutePoseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& posePtr) {
  if (posePtr == nullptr || _firstOdomMsg) {
    ROS_WARN("MaplabIntegrator - nullptr passed to Absolute Pose Factor callback");
    return;
  }

  //Get T_G_C i.e. Camera(C) pose in DARPA(G)
  const geometry_msgs::Pose& p = posePtr->pose.pose;
  const gtsam::Pose3 T_G_C(gtsam::Rot3(p.orientation.w, p.orientation.x, p.orientation.y, p.orientation.z),
                           gtsam::Point3(p.position.x, p.position.y, p.position.z));
  //Calculate IMU(B) pose in DARPA(G)
  gtsam::Pose3 T_G_B = T_G_C * _T_B_C.inverse();

  //Add absolute priors at FIRST key i.e. key=0
  auto t1 = std::chrono::high_resolution_clock::now();
  {
    std::lock_guard<std::mutex> lock(_graphMutex);
    //Create prior factor
    static auto absoluteNoise = gtsam::noiseModel::Diagonal::Sigmas(_absoluteNoise);
    _newFactors.emplace_shared<gtsam::PriorFactor<gtsam::Pose3>>(X(0), T_G_B, absoluteNoise);

    //If first aboslute pose - remove any previous priors at FIRST key
    gtsam::FactorIndices removeFactorIdx;
    if (_firstAbsolutePose) {
      removeFactorIdx.push_back(0);
      _firstAbsolutePose = false;
      // std::cout << "\033[33mABSOLUTE\033[0m First ABSOLUTE POSE received - removing old absolute pose" << std::endl;
    }

    //Update graph
    _graph->update(_newFactors, gtsam::Values(), removeFactorIdx);
    _newFactors.resize(0);

    //Increment total factor count
    incFactorCount();

  }
  auto t2 = std::chrono::high_resolution_clock::now();

  //DEBUG
  std::cout << std::fixed << "\033[33mABSOLUTE\033[0m"
            // <<", ts: " << posePtr->header.stamp.toSec()
            // << ", camera t(x,y,z): " << T_G_C.translation().transpose()  //<< ", RPY(deg): " << lidar_pose.rotation().rpy().transpose() * (180.0 / M_PI)
            // << ", imu t(x,y,z): " << T_G_B.translation().transpose()  //<< ", RPY(deg): " << imu_pose.rotation().rpy().transpose() * (180.0 / M_PI)
            << ", T_G_M t(m):" << _T_G_M.translation().transpose() << ", RPY(deg): " << _T_G_M.rotation().rpy().transpose() * (180.0 / M_PI)
            << ", time(ms): " << std::chrono::duration_cast<std::chrono::milliseconds>(t2 - t1).count()
            << std::endl;
  //DEBUG
}

void MaplabIntegrator::maplabAnchorCallback(const nav_msgs::Path::ConstPtr& pathPtr) {
  //Check if empty message
  if (pathPtr == nullptr || _firstOdomMsg) {
    std::cout << std::fixed << "\033[31mANCHOR\033[0m - Passed nullptr or graph not initialized from odometry yet" << std::endl;
    return;
  }

  std::cout << std::fixed << "\033[36mANCHOR\033[0m - seq:" << pathPtr->header.seq << ", ts: " << pathPtr->header.stamp.toSec() << ", Poses: " << pathPtr->poses.size() << std::endl;

  //Loop through all pose updates and add prior factor on graph nodes
  {
    std::lock_guard<std::mutex> lock(_graphMutex);
    auto t1 = std::chrono::high_resolution_clock::now();
    gtsam::FactorIndices removeFactorIdx;
    for (const geometry_msgs::PoseStamped& pose_msg : pathPtr->poses) {
      //Update timestamp
      const double ts = pose_msg.header.stamp.toSec();

      //Find corresponding key in graph
      gtsam::Key key;
      auto key_itr = _timestampKeyMap.find(ts);
      if (key_itr != _timestampKeyMap.end()) {
        key = key_itr->second; //Save Key
        std::cout << std::fixed << "\033[36mANCHOR\033[0m - Found Key: " << key << ", ts: " << ts;
      } else {
        std::cout << std::fixed << "\033[36mANCHOR\033[0m - Found no closest key for ts: :" << ts << std::endl;
        continue;
      }

      //Only Absolute poses can be attached to Key 0
      if (key == 0)
        continue;

      //Create Update pose
      const auto& p = pose_msg.pose;
      const gtsam::Pose3 T_G_B(gtsam::Rot3(p.orientation.w, p.orientation.x, p.orientation.y, p.orientation.z),
                               gtsam::Point3(p.position.x, p.position.y, p.position.z));

      //Compare if anchor pose on same key has changed
      auto anchor_itr = _keyAnchorPoseMap.find(key);
      if (anchor_itr != _keyAnchorPoseMap.end()) {
        bool equal = anchor_itr->second.equals(T_G_B, 1.0e-02);
        std::cout << ", Equal: " << (equal ? " \033[32mYES\033[0m - SKIP\n" : "\033[31mNO\033[0m");
        if (equal)
          continue;
      }

      //Find if Prior factor exists at Key and get Factor Index
      auto itr = _keyAnchorFactorIdxMap.find(key);
      if (itr != _keyAnchorFactorIdxMap.end()) {
        std::cout << " - Removing Prior with Index: " << itr->second << ", at Key: " << key << std::endl;
        removeFactorIdx.push_back(itr->second);
      } else
        std::cout << " - New Prior Added" << std::endl;

      //Update graph
      static auto anchorNoise = gtsam::noiseModel::Diagonal::Sigmas(_anchorNoise);
      _newFactors.emplace_shared<gtsam::PriorFactor<gtsam::Pose3>>(X(key), T_G_B, anchorNoise);
      _graph->update(_newFactors, gtsam::Values(), removeFactorIdx);
      _newFactors.resize(0);  //Reset
      incFactorCount();

      //Update lookup maps
      updateKeyAnchorFactorIdxMap(key);    //Assosicate index of new prior factor to graph key
      updateKeyAnchorPoseMap(key, T_G_B);  //Associate Anchor pose to key
    }
    auto t2 = std::chrono::high_resolution_clock::now();
    std::cout << "\033[36mANCHOR-UPDATE\033[0m - time(ms): " << std::chrono::duration_cast<std::chrono::milliseconds>(t2 - t1).count() << std::endl;
  }
}

void MaplabIntegrator::maplabSubmapCallback(const nav_msgs::Path::ConstPtr& pathPtr) {
  //Check if empty message
  if (pathPtr == nullptr || _firstOdomMsg) {
    ROS_WARN("MaplabIntegrator - nullptr passed to Submap callback or graph not initialized from odometry yet");
    return;
  }

  //Find corresponding parent key in graph
  const auto& parent_ts = pathPtr->header.stamp.toSec();
  {
    std::lock_guard<std::mutex> lock(_graphMutex);
    auto parent_itr = _timestampKeyMap.find(parent_ts);
    if (parent_itr != _timestampKeyMap.end()) {
      gtsam::Key parent_key = parent_itr->second;
      // std::cout << std::fixed << "\033[34mSUBMAP\033[0m - Parent Key : " << parent_key << ", ts: " << parent_ts;
      //Loop through submap(parent)-submap(child) constraints
      auto t1 = std::chrono::high_resolution_clock::now();
      for (size_t i = 0; i < pathPtr->poses.size(); ++i) {
        const auto& child_ts = pathPtr->poses[i].header.stamp.toSec();
        //Check if child is associated to a key
        auto child_itr = _timestampKeyMap.find(child_ts);
        if (child_itr != _timestampKeyMap.end()) {
          gtsam::Key child_key = child_itr->second;
          // std::cout << std::fixed << ", Child Key : " << child_key << ", ts: " << child_ts << std::endl;

          //Skip if keys are same
          if (child_key == parent_key) {
            ROS_WARN_STREAM("MaplabIntegrator - Same Submap Parent/Child keys, key:" << child_key);
            continue;
          }

          //Check if a previous BetweenFactor exists between two keys
          gtsam::FactorIndices removeFactorIdx;
          int rmIdx = findSubmapFactorIdx(parent_key, child_key, true);
          if (rmIdx != -1) {
            removeFactorIdx.push_back(rmIdx);
            // std::cout << "Factor index to be removed: " << rmIdx << ", between keys: " << parent_key << "/" << child_key << std::endl;
          }

          //Add submap-submap BetweenFactor
          static auto submapNoise = gtsam::noiseModel::Diagonal::Sigmas(_submapNoise);
          const auto& p = pathPtr->poses[i].pose;
          gtsam::Pose3 T_B1B2(gtsam::Rot3(p.orientation.w, p.orientation.x, p.orientation.y, p.orientation.z),
                              gtsam::Point3(p.position.x, p.position.y, p.position.z));
          gtsam::BetweenFactor<gtsam::Pose3> submapBF(X(parent_key), X(child_key), T_B1B2, submapNoise);
          _newFactors.add(submapBF);
          //Update Graph
          _graph->update(_newFactors, gtsam::Values(), removeFactorIdx);
          _newFactors.resize(0);
          incFactorCount();
          updateKeySubmapFactorIdxMap(parent_key, child_key);
        } else {
          // std::cout << std::fixed << ", NO child key found, ts: " << child_ts << std::endl;
          continue;
        }
      }
      auto t2 = std::chrono::high_resolution_clock::now();

      //DEBUG
      std::cout << "\033[34mSUBMAP-UPDATE\033[0m - Constraints added: " << pathPtr->poses.size()
                << ", time(ms): " << std::chrono::duration_cast<std::chrono::milliseconds>(t2 - t1).count()
                << std::endl;
      //DEBUG
    } else {
      std::cout << std::fixed << "\033[34mSUBMAP\033[0m  - Found no key for parent at ts: " << parent_ts << " --- SKIPPING CHILDERN ---" << std::endl;
      return;
    }
  }
}

void MaplabIntegrator::addPriorFactor(const gtsam::Key key, const gtsam::Pose3& pose) {
  //Prior factor noise
  static auto priorNoise = gtsam::noiseModel::Isotropic::Sigma(6, 1e-2);

  //Initial estimate - Use pose as estimate (For prior factors at graph init)
  gtsam::Values estimate;
  estimate.insert(X(key), pose);

  //Add prior factor and update graph
  _newFactors.emplace_shared<gtsam::PriorFactor<gtsam::Pose3>>(X(key), pose, priorNoise);
  _graph->update(_newFactors, estimate);
  _newFactors.resize(0);  //Reset
  incFactorCount();
}

void MaplabIntegrator::addPoseBetweenFactor(const gtsam::Key old_key, const gtsam::Key new_key, const gtsam::Pose3& pose_delta, const gtsam::Pose3& pose_estimate) {
  //Create Pose BetweenFactor
  static auto poseBFNoise = gtsam::noiseModel::Diagonal::Sigmas(_odomNoise);
  gtsam::BetweenFactor<gtsam::Pose3> poseBF(X(old_key), X(new_key), pose_delta, poseBFNoise);

  //Pose estimate
  gtsam::Values estimate;
  estimate.insert(X(new_key), pose_estimate);

  //Add factor and update graph
  _newFactors.add(poseBF);
  _graph->update(_newFactors, estimate);
  _newFactors.resize(0);  //Reset
  incFactorCount();
}

bool MaplabIntegrator::findClosestKeyForTS(const double ts, gtsam::Key* key) const {
  if (key == nullptr || _states.empty()) {
    return false;
  }
  static constexpr double const& eps = 0.2;  // not sure yet
  const double oldest_ts = (*_states.crbegin())->ts() + eps;
  if (ts > oldest_ts) {
    // The requested ts is older than the oldest one in the state vector.
    return false;
  }
  auto comp = [](const StatePtr& lhs, const double ts) { return lhs->ts() + eps < ts; };
  auto it = std::lower_bound(_states.cbegin(), _states.cend(), ts, comp);
  if (it == _states.cend()) {
    return false;
  }

  *key = (*it)->key();
  return true;
}

void MaplabIntegrator::updateGraphResults() {
  //Check if graph has initialized
  if(_firstOdomMsg)
    return;

  //Update results
  gtsam::Values result;
  auto t_0 = std::chrono::high_resolution_clock::now();
  {
    std::lock_guard<std::mutex> lock(_graphMutex);
    result = _graph->calculateBestEstimate();
  }
  auto t_update = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::high_resolution_clock::now() - t_0).count();

  //Update result timestamp
  ros::Time resultTimestamp = ros::Time::now();

  auto t1 = std::chrono::high_resolution_clock::now();
  nav_msgs::Path pathMsg;
  pcl::PointCloud<pcl::PointXYZ>::Ptr mapCloud(new pcl::PointCloud<pcl::PointXYZ>);
  int cloud_count = 0;

  //Loop through result values - Result in absolute frame
  _T_G_M = result.at<gtsam::Pose3>(X(0));
  for (std::size_t i = 0; i < result.size(); ++i) {
    gtsam::Pose3 T_G_B = result.at<gtsam::Pose3>(X(i));

    // Create pose message.
    geometry_msgs::PoseStamped poseMsg;
    poseMsg.header.frame_id = _world_frame;
    poseMsg.header.stamp = resultTimestamp;
    createPoseMessage(T_G_B, &poseMsg);
    pathMsg.poses.emplace_back(poseMsg);

    //Create Map
    if (_pubMap.getNumSubscribers() > 0) {
      auto itr = _keyCloudMap.find(i);
      if (itr != _keyCloudMap.end()) {
        //Get global lidar pose
        gtsam::Pose3 T_G_L = T_G_B * _T_L_B.inverse();
        //Transform cloud
        pcl::PointCloud<pcl::PointXYZ>::Ptr transformedCloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::transformPointCloud(*itr->second, *transformedCloud, T_G_L.matrix());
        *mapCloud += *transformedCloud;
        ++cloud_count;
      }
    }
  }
  pathMsg.header.frame_id = _world_frame;
  pathMsg.header.stamp = resultTimestamp;
  _pubUpdatedPath.publish(pathMsg);

  //Publish Map
  if (_pubMap.getNumSubscribers() > 0) {
    //Downsample Map
    pcl::VoxelGrid<pcl::PointXYZ> voxGrid;
    voxGrid.setLeafSize(0.4, 0.4, 0.4);
    voxGrid.setInputCloud(mapCloud);
    voxGrid.filter(*mapCloud);
    //Publish
    sensor_msgs::PointCloud2 rosCloud;
    pcl::toROSMsg(*mapCloud, rosCloud);
    rosCloud.header.frame_id = _world_frame;
    rosCloud.header.stamp = resultTimestamp;
    _pubMap.publish(rosCloud);
  }

  //Update last optimzed pose
  _T_G_B_opt = result.at<gtsam::Pose3>(X(result.size() - 1));

  //DEBUG
  auto t2 = std::chrono::high_resolution_clock::now();
  std::cout << "\033[36mMaplabIntegrator\033[0m - Graph Update(ms): " << t_update 
            <<", Map Build(ms): " << std::chrono::duration_cast<std::chrono::milliseconds>(t2 - t1).count() 
            // <<", clouds: " << cloud_count << ", points: " << mapCloud->points.size()
            << std::endl;
  //DEBUG
}

void MaplabIntegrator::publishTransforms(const ros::Time& ts) {
  // Absolute(DARPA)-to-RobotMap
  if (_publishWorldtoMapTf) {
    geometry_msgs::TransformStamped T_G_M;
    T_G_M.header.stamp = ts;
    T_G_M.header.frame_id = _world_frame;
    T_G_M.child_frame_id = _map_frame;
    T_G_M.transform.rotation.x = _T_G_M.rotation().toQuaternion().x();
    T_G_M.transform.rotation.y = _T_G_M.rotation().toQuaternion().y();
    T_G_M.transform.rotation.z = _T_G_M.rotation().toQuaternion().z();
    T_G_M.transform.rotation.w = _T_G_M.rotation().toQuaternion().w();
    T_G_M.transform.translation.x = _T_G_M.translation().x();
    T_G_M.transform.translation.y = _T_G_M.translation().y();
    T_G_M.transform.translation.z = _T_G_M.translation().z();
    _tb.sendTransform(T_G_M);
  }

  // Base(IMU)-to-Absolute(DARPA) - Incremental Update
  geometry_msgs::TransformStamped T_G_B_inc;
  T_G_B_inc.header.stamp = ts;
  T_G_B_inc.header.frame_id = _world_frame;
  T_G_B_inc.child_frame_id = _base_frame + "_increment";
  T_G_B_inc.transform.rotation.x = _T_G_B_inc.rotation().toQuaternion().x();
  T_G_B_inc.transform.rotation.y = _T_G_B_inc.rotation().toQuaternion().y();
  T_G_B_inc.transform.rotation.z = _T_G_B_inc.rotation().toQuaternion().z();
  T_G_B_inc.transform.rotation.w = _T_G_B_inc.rotation().toQuaternion().w();
  T_G_B_inc.transform.translation.x = _T_G_B_inc.translation().x();
  T_G_B_inc.transform.translation.y = _T_G_B_inc.translation().y();
  T_G_B_inc.transform.translation.z = _T_G_B_inc.translation().z();
  _tb.sendTransform(T_G_B_inc);
  //PoseStamped message
  if (_pubIncPoseStamped.getNumSubscribers() > 0) {
    geometry_msgs::PoseStamped T_G_B_inc_poseMsg;
    convertTransfromToPose(T_G_B_inc, T_G_B_inc_poseMsg);
    _pubIncPoseStamped.publish(T_G_B_inc_poseMsg);
  }

  // Base(IMU)-to-Absolute(DARPA) - Optimized
  geometry_msgs::TransformStamped T_G_B_opt;
  T_G_B_opt.header.stamp = ts;
  T_G_B_opt.header.frame_id = _world_frame;
  T_G_B_opt.child_frame_id = _base_frame + "_optimized";
  T_G_B_opt.transform.rotation.x = _T_G_B_opt.rotation().toQuaternion().x();
  T_G_B_opt.transform.rotation.y = _T_G_B_opt.rotation().toQuaternion().y();
  T_G_B_opt.transform.rotation.z = _T_G_B_opt.rotation().toQuaternion().z();
  T_G_B_opt.transform.rotation.w = _T_G_B_opt.rotation().toQuaternion().w();
  T_G_B_opt.transform.translation.x = _T_G_B_opt.translation().x();
  T_G_B_opt.transform.translation.y = _T_G_B_opt.translation().y();
  T_G_B_opt.transform.translation.z = _T_G_B_opt.translation().z();
  _tb.sendTransform(T_G_B_opt);
  //PoseStamped message
  if (_pubOptPoseStamped.getNumSubscribers() > 0) {
    geometry_msgs::PoseStamped T_G_B_opt_poseMsg;
    convertTransfromToPose(T_G_B_opt, T_G_B_opt_poseMsg);
    _pubOptPoseStamped.publish(T_G_B_opt_poseMsg);
  }
}

bool MaplabIntegrator::createPoseMessage(const gtsam::Pose3& pose, geometry_msgs::PoseStamped* poseMsg) const {
  if (poseMsg == nullptr) {
    return false;
  }
  poseMsg->pose.position.x = pose.translation().x();
  poseMsg->pose.position.y = pose.translation().y();
  poseMsg->pose.position.z = pose.translation().z();
  poseMsg->pose.orientation.x = pose.rotation().toQuaternion().x();
  poseMsg->pose.orientation.y = pose.rotation().toQuaternion().y();
  poseMsg->pose.orientation.z = pose.rotation().toQuaternion().z();
  poseMsg->pose.orientation.w = pose.rotation().toQuaternion().w();

  return true;
}

void MaplabIntegrator::convertTransfromToPose(const geometry_msgs::TransformStamped& t, geometry_msgs::PoseStamped& p){
  //Header
  p.header = t.header;
  //Translation
  p.pose.position.x = t.transform.translation.x;
  p.pose.position.y = t.transform.translation.y;
  p.pose.position.z = t.transform.translation.z;
  //Rotation
  p.pose.orientation = t.transform.rotation;
}

void MaplabIntegrator::updateKeySubmapFactorIdxMap(const gtsam::Key parent_key, const gtsam::Key child_key) {
  size_t factor_idx = getFactorCount() - 1;
  _keySubmapFactorIdxMap[parent_key].emplace(factor_idx);
  _keySubmapFactorIdxMap[child_key].emplace(factor_idx);

  //DEBUG
  // std::cout << "\033[34mSUBMAP\033[0m NEW Factor added at Index: " << factor_idx << ", between keys: " << parent_key << "/" << child_key << std::endl;
}

int MaplabIntegrator::findSubmapFactorIdx(const gtsam::Key parent_key, const gtsam::Key child_key, bool erase) {
  //Get sets of constraint indices at graph keys
  auto& p_set = _keySubmapFactorIdxMap[parent_key];
  auto& c_set = _keySubmapFactorIdxMap[child_key];

  //Find common constraint index between two graph Keys i.e. a BetweenFactor should have same factor index at two graph keys(nodes)
  std::set<size_t> result;
  std::set_intersection(p_set.begin(), p_set.end(),
                        c_set.begin(), c_set.end(),
                        std::inserter(result, result.begin()));

  //Check if success
  int output = -1;
  if (result.size() == 1) {
    output = *result.begin();

    //Remove common constraint indices from both sets
    if (erase) {
      p_set.erase(output);
      c_set.erase(output);
    }
  } else if (result.size() > 1) {
    ROS_WARN("MaplabIntegrator - Submaps connected by mutliple between factors");
    std::cout << "Indices found: ";
    for (auto& e : result)
      std::cout << e << " ";
    std::cout << ", between keys: " << parent_key << "/" << child_key << std::endl;
  }

  return output;
}
}  // namespace loam
