#include "loam/MaplabIntegrator.hpp"

#include <algorithm>

#include <gtsam/navigation/GPSFactor.h>

namespace loam {

bool MaplabIntegrator::setup(ros::NodeHandle& node, ros::NodeHandle& privateNode) {
  //Subscribers
  _subLaserMapOdom = node.subscribe<nav_msgs::Odometry>("/loam_recorded_odometry", 10, &MaplabIntegrator::lidarOdomCallback, this);
  _subAbsolutePose = node.subscribe<geometry_msgs::PoseWithCovarianceStamped>("/anymal/absolute_reference", 2, &MaplabIntegrator::absolutePoseCallback, this);
  _subMaplabAnchor = node.subscribe<nav_msgs::Path>("/graph_client/anchor_nodes", 2, &MaplabIntegrator::maplabAnchorCallback, this);
  _subMaplabSubmap = node.subscribe<nav_msgs::Path>("/graph_client/intra_constraints", 2, &MaplabIntegrator::maplabSubmapCallback, this);

  //Publisher
  _pubGraphPath = node.advertise<nav_msgs::Path>("/incremental_trajectory", 5);
  _pubUpdatedPath = node.advertise<nav_msgs::Path>("/updated_trajectory", 5);
  _pathMsg.poses.reserve(20000);  //1 hour path @ 5Hz ~ 18000
  _pathMsg.header.frame_id = "cerberus";

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
  _params.findUnusedFactorSlots = false;
  _params.setEnablePartialRelinearizationCheck(true);
  _params.setEnableDetailedResults(false);
  //Initialize factor graph object
  _graph = std::make_shared<gtsam::ISAM2>(_params);
  _graph->params().print("Graph Integrator - Parameters:");
  std::cout << "Graph Integrator - Pose Between Factor Noise - RPY(rad): " << _poseNoise[0] << "," << _poseNoise[1] << "," << _poseNoise[2]
            << ", XYZ(m): " << _poseNoise[3] << "," << _poseNoise[4] << "," << _poseNoise[5] << std::endl;

  //Intialize transforms //TODO: fixed for now
  Eigen::Matrix4d T_B_L;
  T_B_L << 1, 0, 0, 0.005303,
      0, 1, 0, 0.037340,
      0, 0, 1, 0.063319,
      0, 0, 0, 1;
  _T_B_L = gtsam::Pose3(T_B_L);

  Eigen::Matrix4d T_B_C;
  T_B_C << -0.9999962982, -0.000530768, 0.0026686827, 0.0471397497,
      -0.0026684873, -0.0003684325, -0.9999963717, -0.0778675546,
      0.0005317493, -0.9999997913, 0.0003670147, 0.0260845632,
      0.0, 0.0, 0.0, 1.0;
  _T_B_C = gtsam::Pose3(T_B_C);

  return true;
}

void MaplabIntegrator::lidarOdomCallback(const nav_msgs::Odometry::ConstPtr& odomPtr) {
  if (odomPtr == nullptr) {
    std::cerr << std::fixed << "\033[35mODOMETRY\033[0m - Passed nullptr in the map odom callback. " << std::endl;
    return;
  }

  //Current timestamp and LiDAR pose
  const double ts = odomPtr->header.stamp.toSec();
  const auto& p = odomPtr->pose.pose;
  const gtsam::Pose3 lidar_pose(gtsam::Rot3(p.orientation.w, p.orientation.x, p.orientation.y, p.orientation.z),
                                gtsam::Point3(p.position.x, p.position.y, p.position.z));

  //Add delta pose factor to graph
  gtsam::Key new_key;
  gtsam::Values result;
  {
    //If first lidar pose initialize graph
    std::lock_guard<std::mutex> lock(_graphMutex);
    if (_firstLidarPose) {
      //Initialize
      new_key = _state_key;
      addPriorFactor(_state_key, gtsam::Pose3(), true);
      _firstLidarPose = false;
    } else {
      //Add pose delta
      auto old_key = _states.back()->key();
      new_key = newStateKey();
      gtsam::Pose3 pose_delta = _lastLidarPose.between(lidar_pose);
      gtsam::Pose3 pose_estimate = _T_B_L.compose(lidar_pose);
      addPoseBetweenFactor(old_key, new_key, pose_delta, pose_estimate);
    }
    //Update Timestamp-Key Map
    _timestampKeyMap[ts] = new_key;
    //Get updated result
    result = _graph->calculateEstimate();
  }

  //Update vector of states in (B) frame
  StatePtr state = std::make_unique<State>(new_key, ts, gtsam::NavState(result.at<gtsam::Pose3>(X(new_key)), gtsam::Vector3(0, 0, 0)));
  _states.emplace_back(std::move(state));
  const auto& imu_pose = _states.back()->navState().pose();

  //Update last LiDAR pose
  _lastLidarPose = lidar_pose;

  //Publish Graph path
  //Pose Message
  static size_t poseMsgCount = 0;
  geometry_msgs::PoseStamped poseMsg;
  poseMsg.header.frame_id = "alphasense_imu";
  poseMsg.header.stamp = odomPtr->header.stamp;
  poseMsg.header.seq = ++poseMsgCount;
  createPoseMessage(imu_pose, &poseMsg);

  //Node Message
  _pathMsg.header.stamp = poseMsg.header.stamp;
  _pathMsg.poses.emplace_back(std::move(poseMsg));

  //Publish Path
  _pubGraphPath.publish(_pathMsg);

  // DEBUG
  // std::cout << std::fixed << "\033[31mODOMETRY\033[0m - key: " << new_key
  //           << ", ts: " << odomPtr->header.stamp.toSec()
  //           << ", lidar t(x,y,z): " << lidar_pose.translation().transpose() //<< ", RPY(deg): " << lidar_pose.rotation().rpy().transpose() * (180.0 / M_PI)
  //           << ", imu t(x,y,z): " << imu_pose.translation().transpose() //<< ", RPY(deg): " << imu_pose.rotation().rpy().transpose() * (180.0 / M_PI)
  //           << std::endl;

  std::cout << std::fixed << "\033[35mODOMETRY\033[0m - key: " << new_key
            << ", ts: " << odomPtr->header.stamp.toSec()
            << ", Factors: " << _graph->getFactorsUnsafe().size()
            << std::endl;
  // DEBUG
}

void MaplabIntegrator::absolutePoseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& posePtr) {
  if (posePtr == nullptr) {
    return;
  }

  //Get T_G_C i.e. Camera(C) pose in DARPA(G)
  const geometry_msgs::Pose& p = posePtr->pose.pose;
  const gtsam::Pose3 T_G_C(gtsam::Rot3(p.orientation.w, p.orientation.x, p.orientation.y, p.orientation.z),
                           gtsam::Point3(p.position.x, p.position.y, p.position.z));
  //Calculate IMU(B) pose in DARPA(G)
  gtsam::Pose3 T_G_B = T_G_C * _T_B_C.inverse();

  //Add absolute priors at FIRST key i.e. key=0
  {
    std::lock_guard<std::mutex> lock(_graphMutex);
    //Create prior factor
    static auto absoluteNoise = gtsam::noiseModel::Isotropic::Sigma(6, 1e-6);  // rad,rad,rad,m, m, m
    _newFactors.emplace_shared<gtsam::PriorFactor<gtsam::Pose3>>(X(0), T_G_B, absoluteNoise);

    gtsam::FactorIndices removeFactorIdx;
    if (_firstAbsolutePose) {
      removeFactorIdx.push_back(0);
      _firstAbsolutePose = false;
      std::cout << "\033[33mABSOLUTE\033[0m First ABSOLUTE POSE received - removing old absolute pose" << std::endl;
    }

    //Update graph
    _graph->update(_newFactors, gtsam::Values(), removeFactorIdx);
    _newFactors.resize(0);
  }

  //DEBUG
  std::cout << std::fixed << "\033[33mABSOLUTE\033[0m, ts: " << posePtr->header.stamp.toSec()
            << ", camera t(x,y,z): " << T_G_C.translation().transpose()  //<< ", RPY(deg): " << lidar_pose.rotation().rpy().transpose() * (180.0 / M_PI)
            << ", imu t(x,y,z): " << T_G_B.translation().transpose()     //<< ", RPY(deg): " << imu_pose.rotation().rpy().transpose() * (180.0 / M_PI)
            << std::endl;
  //DEBUG
}

void MaplabIntegrator::maplabAnchorCallback(const nav_msgs::Path::ConstPtr& pathPtr) {
  //Check if empty message
  if (pathPtr == nullptr) {
    std::cerr << std::fixed << "\033[31mANCHOR\033[0m - Passed nullptr in graph update callback." << std::endl;
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
      // This is just a precaution as I need to verify whether the timestamp match perfectly.
      // Consequently, I would leave the vector of states for now to be more flexible.
      gtsam::Key key;
      if (!findClosestKeyForTS(ts, &key)) {
        std::cout << std::fixed << "\033[36mANCHOR\033[0m - Found no closest key for ts: :" << ts << std::endl;
        continue;
      } else
        std::cout << std::fixed << "\033[36mANCHOR\033[0m - Found Key: " << key << ", ts: " << ts;

      //Skip Anchor on Key 0 as it has Absolute poses attached to it
      if (key == 0) {
        std::cout << std::fixed << " - Skipping Key 0" << std::endl;
        continue;
      }

      //Create Update pose
      const auto& p = pose_msg.pose;
      const gtsam::Pose3 T_G_B(gtsam::Rot3(p.orientation.w, p.orientation.x, p.orientation.y, p.orientation.z),
                               gtsam::Point3(p.position.x, p.position.y, p.position.z));
      const gtsam::Pose3 T_M_B = _T_G_M.inverse() * T_G_B;

      //Check if key already exists
      auto anchor_itr = _keyAnchorPoseMap.find(key);
      //If match found compare difference
      if (anchor_itr != _keyAnchorPoseMap.end()) {
        bool equal = anchor_itr->second.equals(T_M_B, 1.0e-02);
        std::cout << ", t(old): " << anchor_itr->second.translation().transpose()
                  << ", t(new): " << T_M_B.translation().transpose()
                  << ", Equal: " << (equal ? " \033[32mYES\033[0m" : "\033[31mNO\033[0m") << std::endl;
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

      //Create prior factor
      static auto anchorNoise = gtsam::noiseModel::Isotropic::Sigma(6, 1e-5);  // rad,rad,rad,m, m, m
      _newFactors.emplace_shared<gtsam::PriorFactor<gtsam::Pose3>>(X(key), T_M_B, anchorNoise);

      //Update maps for lookup //NOTE Need to remove/add factor each time to track factor indices
      _graph->update(_newFactors, gtsam::Values(), removeFactorIdx);
      _newFactors.resize(0);               //Reset
      updateKeyAnchorFactorIdxMap(key);    //Assosicate index of new prior factor to graph key
      updateKeyAnchorPoseMap(key, T_M_B);  //Associate Anchor pose to key
    }
    auto t2 = std::chrono::high_resolution_clock::now();
    std::cout << "\033[36mANCHOR-UPDATE\033[0m - time(ms): " << std::chrono::duration_cast<std::chrono::milliseconds>(t2 - t1).count() << std::endl;

    //DEBUG //TODO Move outside Mutex
    visualizeOnboardGraph();
  }
}

void MaplabIntegrator::maplabSubmapCallback(const nav_msgs::Path::ConstPtr& pathPtr) {
  //Check if empty message
  if (pathPtr == nullptr) {
    std::cerr << std::fixed << "\033[34mSUBMAP\033[0m - Passed nullptr in graph update callback." << std::endl;
    return;
  }

  //Find corresponding key in graph
  const auto& parent_ts = pathPtr->header.stamp.toSec();
  gtsam::Key parent_key;
  if (!findClosestKeyForTS(parent_ts, &parent_key)) {
    std::cout << std::fixed << "\033[34mSUBMAP\033[0m  - Found no closest key for parent --- SKIPPING CHILDERN, ts:" << parent_ts << std::endl;
    return;
  } else {
    std::cout << std::fixed << "\033[34mSUBMAP\033[0m - Parent Key : " << parent_key << ", ts: " << parent_ts;
    //Loop through submap-submap constraints
    {
      std::lock_guard<std::mutex> lock(_graphMutex);
      auto t1 = std::chrono::high_resolution_clock::now();
      for (size_t i = 0; i < pathPtr->poses.size(); ++i) {
        const auto& child_ts = pathPtr->poses[i].header.stamp.toSec();
        gtsam::Key child_key;
        //Check if child is associated to a key
        if (!findClosestKeyForTS(child_ts, &child_key)) {
          std::cout << std::fixed << " but found NO key for child at: " << child_ts << std::endl;
          continue;
        } else {
          const auto& p = pathPtr->poses[i].pose;
          std::cout << std::fixed << ", Child Key : " << child_key << ", ts: " << child_ts
                    << ", detla_t(x,y,z): " << p.position.x << "," << p.position.y << "," << p.position.z << std::endl;

          //Check if a previous BetweenFactor exists between two keys
          gtsam::FactorIndices removeFactorIdx;
          int rmIdx = findSubmapFactorIdx(parent_key, child_key, true);
          if (rmIdx != -1) {
            removeFactorIdx.push_back(rmIdx);
            std::cout << "Factor index to be removed: " << rmIdx << ", between keys: " << parent_key << "/" << child_key << std::endl;
          }

          //Add BetweenFactor
          static auto submapNoise = gtsam::noiseModel::Isotropic::Sigma(6, 1e-3);  // rad,rad,rad,m,m,m
          const gtsam::Pose3 T_Parent_Child(gtsam::Rot3(p.orientation.w, p.orientation.x, p.orientation.y, p.orientation.z),
                                            gtsam::Point3(p.position.x, p.position.y, p.position.z));
          gtsam::BetweenFactor<gtsam::Pose3> submapBF(X(parent_key), X(child_key), T_Parent_Child, submapNoise);
          _newFactors.add(submapBF);
          //Update Graph
          _graph->update(_newFactors, gtsam::Values(), removeFactorIdx);
          _newFactors.resize(0);  //Reset
          updateKeySubmapFactorIdxMap(parent_key, child_key);
        }
      }
      auto t2 = std::chrono::high_resolution_clock::now();
      std::cout << "\033[34mSUBMAP-UPDATE\033[0m - time(ms): " << std::chrono::duration_cast<std::chrono::milliseconds>(t2 - t1).count() << std::endl;
      //DEBUG //TODO Move outside Mutex
      visualizeOnboardGraph();
    }
  }
  std::cout << "----------------------------------------------------------------------" << std::endl;
}

void MaplabIntegrator::addPriorFactor(const gtsam::Key key, const gtsam::Pose3& pose, const bool addEstimate) {
  //Prior factor noise
  static auto poseNoise = gtsam::noiseModel::Diagonal::Sigmas((gtsam::Vector(6) << 1e-2, 1e-2, 1e-2, 1e-2, 1e-2, 1e-2).finished());  // rad,rad,rad,m, m, m

  //Initial estimate - Use pose as estimate (For prior factors at graph init)
  gtsam::Values estimate;
  if (addEstimate)
    estimate.insert(X(key), pose);

  //Add prior factor and update graph
  _newFactors.emplace_shared<gtsam::PriorFactor<gtsam::Pose3>>(X(key), pose, poseNoise);  //POSE - PriorFactor format is (key,value,matrix) value is same type as type of PriorFactor
  _graph->update(_newFactors, estimate);
  _newFactors.resize(0);  //Reset
}

void MaplabIntegrator::addPoseBetweenFactor(const gtsam::Key old_key, const gtsam::Key new_key, const gtsam::Pose3& pose_delta, const gtsam::Pose3& pose_estimate) {
  //Create Pose BetweenFactor
  static auto poseNoiseModel = gtsam::noiseModel::Diagonal::Sigmas((gtsam::Vector(6) << _poseNoise[0], _poseNoise[1], _poseNoise[2], _poseNoise[3], _poseNoise[4], _poseNoise[5]).finished());  // rad,rad,rad,m,m,m
  gtsam::BetweenFactor<gtsam::Pose3> poseBF(X(old_key), X(new_key), pose_delta, poseNoiseModel);

  //Pose estimate
  gtsam::Values estimate;
  estimate.insert(X(new_key), pose_estimate);

  //Add factor and update graph
  _newFactors.add(poseBF);
  _graph->update(_newFactors, estimate);
  _newFactors.resize(0);  //Reset
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
  auto comp = [eps](const StatePtr& lhs, const double ts) { return lhs->ts() + eps < ts; };
  auto it = std::lower_bound(_states.cbegin(), _states.cend(), ts, comp);
  if (it == _states.cend()) {
    return false;
  }

  *key = (*it)->key();
  return true;
}

void MaplabIntegrator::visualizeOnboardGraph() const {
  // This is just a quick publisher to visualize the graph in RViz.
  // It already publishes the graph in the correct frames and therefore,
  // is quite convenient for debugging.
  gtsam::Values result = _graph->calculateEstimate();
  const std::size_t n_nodes = result.size();
  const std::size_t n_states = _states.size();
  if (n_nodes != n_states) {
    std::cerr << "Graph nodes don't equal states: " << n_nodes << " vs. " << n_states << std::endl;
    return;
  }
  nav_msgs::Path pathMsg;
  for (std::size_t i = 0u; i < n_states; ++i) {
    const gtsam::Key& key = _states[i]->key();
    gtsam::Pose3 T_M_B = result.at<gtsam::Pose3>(X(key));
    gtsam::Pose3 T_G_B = _T_G_M * T_M_B;

    // Create pose message.
    geometry_msgs::PoseStamped poseMsg;
    poseMsg.header.frame_id = "alphasense_imu";
    poseMsg.header.stamp.fromSec(_states[i]->ts());
    createPoseMessage(T_G_B, &poseMsg);

    pathMsg.poses.emplace_back(poseMsg);
  }
  pathMsg.header.frame_id = "map";
  pathMsg.header.stamp = ros::Time::now();
  _pubUpdatedPath.publish(pathMsg);
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

void MaplabIntegrator::printKeyFactorIdxMap() {
  std::cout << "--- Printing Key-PriorFactorIndex Map ---\n";
  for (auto itr = _keyAnchorFactorIdxMap.begin(); itr != _keyAnchorFactorIdxMap.end(); ++itr)
    std::cout << "Key: " << itr->first << ", FactorIdx: " << itr->second << std::endl;
}

void MaplabIntegrator::updateKeySubmapFactorIdxMap(const gtsam::Key parent_key, const gtsam::Key child_key) {
  _keySubmapFactorIdxMap[parent_key].emplace(_graph->getFactorsUnsafe().size() - 1);
  _keySubmapFactorIdxMap[child_key].emplace(_graph->getFactorsUnsafe().size() - 1);
  std::cout << "\033[34mSUBMAP\033[0m NEW Factor added at Index: " << _graph->getFactorsUnsafe().size() - 1
            << ", between keys: " << parent_key << "/" << child_key << std::endl;
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
  if (result.size() > 0) {
    output = *result.begin();

    //CUSTOMIZATION
    std::cout << "\033[34mSUBMAP\033[0m Indices found: ";
    for (auto& e : result)
      std::cout << e << ",";
    std::cout << ", between keys: " << parent_key << "/" << child_key << std::endl;
    //CUSTOMIZATION

    //Remove common constraint indices from both sets
    if (erase) {
      p_set.erase(output);
      c_set.erase(output);
    }
  }

  return output;
}
}  // namespace loam
