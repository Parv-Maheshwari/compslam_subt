#include "loam/GraphIntegrator.hpp"

#include <algorithm>

#include <gtsam/navigation/GPSFactor.h>

namespace loam {

bool GraphIntegrator::setup(ros::NodeHandle& node, ros::NodeHandle& privateNode) {
  //Subscribers
  _subLaserMapOdom = node.subscribe<nav_msgs::Odometry>("/loam_recorded_odometry", 10, &GraphIntegrator::laserMapOdomCallback, this);
  _subMaplabAnchor = node.subscribe<nav_msgs::Path>("/graph_client/anchor_nodes", 2, &GraphIntegrator::maplabAnchorCallback, this);
  _subAbsolutePose = node.subscribe<geometry_msgs::PoseWithCovarianceStamped>("/anymal/absolute_reference", 2, &GraphIntegrator::absolutePoseCallback, this);

  //Publisher
  _pubGraphPath = node.advertise<nav_msgs::Path>("/trajectory_path", 5);
  _pubUpdatedPath = node.advertise<nav_msgs::Path>("/updated_trajectory", 5);
  _pathMsg.poses.reserve(20000);  //1 hour path @ 5Hz ~ 18000
  _pathMsg.header.frame_id = "cerberus";

  //Set factor graph params
  _params.optimizationParams = gtsam::ISAM2GaussNewtonParams();
  _params.factorization = gtsam::ISAM2Params::CHOLESKY;  //CHOLESKY:Fast but non-stable //QR:Slower but more stable in poorly conditioned problems
  gtsam::FastMap<char, gtsam::Vector> relinTh;           //Set graph relinearization thresholds - must be lower case letters, check:gtsam::symbol_shorthand
  relinTh['x'] = (gtsam::Vector(6) << 0.05, 0.05, 0.05, 0.05, 0.05, 0.05).finished();
  _params.relinearizeThreshold = relinTh;
  _params.setEnableRelinearization(true);
  _params.setRelinearizeSkip(1);
  _params.setCacheLinearizedFactors(false);
  _params.setEvaluateNonlinearError(false);
  _params.findUnusedFactorSlots = false;
  _params.setEnablePartialRelinearizationCheck(true);
  _params.setEnableDetailedResults(false);
  //Initialize factor graph
  _graph = std::make_shared<gtsam::ISAM2>(_params);
  _graph->params().print("Graph Integrator - Parameters:");
  std::cout << "Graph Integrator - Pose Between Factor Noise - RPY(rad): " << _poseNoise[0] << "," << _poseNoise[1] << "," << _poseNoise[2]
            << ", XYZ(m): " << _poseNoise[3] << "," << _poseNoise[4] << "," << _poseNoise[5] << std::endl;

  //Intialize transforms //NOTE: fixed for now
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

void GraphIntegrator::laserMapOdomCallback(const nav_msgs::Odometry::ConstPtr& odomPtr) {
  if (odomPtr == nullptr) {
    std::cerr << std::fixed << "\033[31mODOMETRY\033[0m - Passed nullptr in the map odom callback. " << std::endl;
    return;
  }
  /*
  std::cout << std::fixed << "\033[31mODOMETRY\033[0m - ts: " << odomPtr->header.stamp.toSec()
            << ", t(x,y,z): " << odomPtr->pose.pose.position.x << "," << odomPtr->pose.pose.position.y << "," << odomPtr->pose.pose.position.z << std::endl;
            */

  //New timestampd and pose
  const double ts = odomPtr->header.stamp.toSec();
  const auto& p = odomPtr->pose.pose;
  const gtsam::Pose3 pose(gtsam::Rot3(p.orientation.w, p.orientation.x, p.orientation.y, p.orientation.z),
                          gtsam::Point3(p.position.x, p.position.y, p.position.z));
  //Build key timestamp map

  //Add to Graph
  gtsam::Key new_key;
  auto t1 = std::chrono::high_resolution_clock::now();
  if (!_isGraphInitialized) {
    new_key = _state_key;  //start at zero
    addPriorFactor(new_key, pose);
    _isGraphInitialized = true;
  } else {
    auto old_key = _states.back()->key();
    new_key = newStateKey();
    addPoseBetweenFactor(old_key, new_key, pose);
  }
  //Get result - LiDAR frame
  gtsam::Values result = _graph->calculateEstimate();
  StatePtr state = std::make_unique<State>(new_key, ts, gtsam::NavState(result.at<gtsam::Pose3>(X(new_key)), gtsam::Vector3(0, 0, 0)));
  _states.emplace_back(std::move(state));
  auto t2 = std::chrono::high_resolution_clock::now();

  //Convert to Alphasense IMU frame
  gtsam::Pose3 T_M_L = _states.back()->navState().pose();
  gtsam::Pose3 T_M_B = T_M_L * _T_B_L.inverse();

  //Publish Graph path
  //Pose Message
  static size_t poseMsgCount = 0;
  geometry_msgs::PoseStamped poseMsg;
  poseMsg.header.frame_id = "alphasense_imu";
  poseMsg.header.stamp = odomPtr->header.stamp;
  poseMsg.header.seq = ++poseMsgCount;
  createPoseMessage(T_M_B, &poseMsg);

  //Node Message
  _pathMsg.header.stamp = poseMsg.header.stamp;
  _pathMsg.poses.emplace_back(std::move(poseMsg));

  //Publish Path
  _pubGraphPath.publish(_pathMsg);

  // DEBUG
  /*
  std::cout << "lidar_pose t(x,y,z): " << T_M_L.translation().transpose() << ", RPY(deg): " << T_M_L.rotation().rpy().transpose() * (180.0 / M_PI)
            << "\nimu_pose  t(x,y,z): " << T_M_B.translation().transpose() << ", RPY(deg): " << T_M_B.rotation().rpy().transpose() * (180.0 / M_PI)
            << ", time(ms): " << std::chrono::duration_cast<std::chrono::milliseconds>(t2 - t1).count() << std::endl;
            */
  visualizeOnboardGraph();
  // DEBUG
}

void GraphIntegrator::maplabAnchorCallback(const nav_msgs::Path::ConstPtr& pathPtr) {
  if (pathPtr == nullptr) {
    std::cerr << std::fixed << "\033[31mANCHOR\033[0m - Passed nullptr in graph update callback." << std::endl;
    return;
  }
  std::cout << std::fixed << "\033[36mANCHOR\033[0m - seq:" << pathPtr->header.seq
            << ", ts: " << pathPtr->header.stamp.toSec()
            << ", Poses: " << pathPtr->poses.size() << std::endl;
  return;
  const std::vector<geometry_msgs::PoseStamped>& poses = pathPtr->poses;
  for (const geometry_msgs::PoseStamped& pose_msg : poses) {
    const double ts = pose_msg.header.stamp.toSec();
    gtsam::Key key;
    // This is just a precaution as I need to verify whether the timestamp match perfectly.
    // Consequently, I would leave the vector of states for now to be more flexible.
    if (!findClosestKeyForTS(ts, &key)) {
      std::cout << std::fixed << "\033[36mANCHOR\033[0m - Found no closest key for ts: :" << ts << std::endl;
      continue;
    }
    const auto& p = pose_msg.pose;
    const gtsam::Pose3 T_G_B(gtsam::Rot3(p.orientation.w, p.orientation.x, p.orientation.y, p.orientation.z),
                             gtsam::Point3(p.position.x, p.position.y, p.position.z));
    const gtsam::Pose3 T_M_B = _T_G_M.inverse() * T_G_B;
    //addPriorFactor(key, T_M_B, false);
    auto poseNoise = gtsam::noiseModel::Diagonal::Sigmas((gtsam::Vector(6) << 1e-1, 1e-1, 1e-1, 1e-1, 1e-1, 1e-1).finished());  // rad,rad,rad,m, m, m
    //Create prior factors
    _newFactors.resize(0);
    _newFactors.emplace_shared<gtsam::PriorFactor<gtsam::Pose3>>(X(key), T_M_B, poseNoise);

    //Update graph
    gtsam::Values estimate;
    _graph->update(_newFactors, estimate);
    _newFactors.resize(0);  //Reset
    break;
  }
}

void GraphIntegrator::absolutePoseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& posePtr) {
  if (posePtr == nullptr) {
    return;
  }
  const geometry_msgs::Pose& p = posePtr->pose.pose;
  const gtsam::Pose3 T_G_C(gtsam::Rot3(p.orientation.w, p.orientation.x, p.orientation.y, p.orientation.z),
                           gtsam::Point3(p.position.x, p.position.y, p.position.z));
  const double ts = posePtr->header.stamp.toSec();
  gtsam::Key key;
  if (!findClosestKeyForTS(ts, &key)) {
    std::cout << std::fixed << "\033[36mANCHOR\033[0m - Found no closest key for ts: :" << ts << std::endl;
    return;
  }
  gtsam::Values result = _graph->calculateEstimate();
  gtsam::Pose3 T_M_B = result.at<gtsam::Pose3>(X(key));
  gtsam::Pose3 T_G_B = T_G_C * _T_B_C.inverse();

  // Outlier rejection. This is a very quick and dirty way to do it.
  // The absolute poses are very noisy and need to be checked.
  // We'll have to find something better than this in the future.
  // Perhaps average?
  if (p.position.z > 0.45 && p.position.z <= 0.8) {
    _T_G_M = T_G_B * T_M_B.inverse();
  }
}

bool GraphIntegrator::addPriorFactor(const gtsam::Key key, const gtsam::Pose3& pose, const bool addEstimate) {
  //Prior factor noise
  auto poseNoise = gtsam::noiseModel::Diagonal::Sigmas((gtsam::Vector(6) << 1e-3, 1e-3, 1e-3, 1e-6, 1e-6, 1e-6).finished());  // rad,rad,rad,m, m, m
  //Create prior factors
  _newFactors.resize(0);
  _newFactors.emplace_shared<gtsam::PriorFactor<gtsam::Pose3>>(X(key), pose, poseNoise);  //POSE - PriorFactor format is (key,value,matrix) value is same type as type of PriorFactor

  //Initial estimate
  gtsam::Values estimate;
  if (addEstimate) {
    estimate.insert(X(key), pose);
  }

  //Update graph
  _graph->update(_newFactors, estimate);
  _newFactors.resize(0);  //Reset
}

bool GraphIntegrator::addPoseBetweenFactor(const gtsam::Key old_key, const gtsam::Key new_key, const gtsam::Pose3& pose) {
  //Calculate Pose delta
  gtsam::Pose3 pose_delta = _states.back()->navState().pose().between(pose);
  //Create Pose BetweenFactor and add
  auto poseNoiseModel = gtsam::noiseModel::Diagonal::Sigmas((gtsam::Vector(6) << _poseNoise[0], _poseNoise[1], _poseNoise[2], _poseNoise[3], _poseNoise[4], _poseNoise[5]).finished());  // rad,rad,rad,m,m,m
  gtsam::BetweenFactor<gtsam::Pose3> poseBetweenFactor(X(old_key), X(new_key), pose_delta, poseNoiseModel);
  _newFactors.resize(0);  //Reset
  _newFactors.add(poseBetweenFactor);
  //Estimate
  gtsam::Values estimate;
  estimate.insert(X(new_key), pose);
  //Update graph
  _graph->update(_newFactors, estimate);
  _newFactors.resize(0);  //Reset
}

bool GraphIntegrator::findClosestKeyForTS(const double ts, gtsam::Key* key) const {
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

void GraphIntegrator::visualizeOnboardGraph() const {
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

bool GraphIntegrator::createPoseMessage(const gtsam::Pose3& pose, geometry_msgs::PoseStamped* poseMsg) const {
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

}  // namespace loam
