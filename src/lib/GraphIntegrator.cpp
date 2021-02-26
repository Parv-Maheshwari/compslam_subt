#include "loam/GraphIntegrator.hpp"

namespace loam {
bool GraphIntegrator::setup(ros::NodeHandle& node, ros::NodeHandle& privateNode) {
  //Subscribers
  _subLaserMapOdom = node.subscribe<nav_msgs::Odometry>("/aft_mapped_to_init", 10, &GraphIntegrator::laserMapOdomCallback, this);
  _subMaplabPath = node.subscribe<nav_msgs::Path>("/maplab_server/global_sparse_graph", 2, &GraphIntegrator::maplabPathCallback, this);

  //Publisher
  _pubGraphPath = node.advertise<maplab_msgs::Trajectory>("/trajectory", 5);
  _trajectoryMsg.nodes.reserve(20000);  //1 hour path @ 5Hz ~ 18000

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
  Eigen::Matrix4d T_BL;
  T_BL << 1, 0, 0, 0.005303,
      0, 1, 0, 0.037340,
      0, 0, 1, 0.063319,
      0, 0, 0, 1;
  _lidar2imu = gtsam::Pose3(T_BL);

  return true;
}

void GraphIntegrator::laserMapOdomCallback(const nav_msgs::Odometry::ConstPtr& odomPtr) {
  std::cout << std::fixed << "\033[31mODOMETRY\033[0m - ts: " << odomPtr->header.stamp.toSec()
            << ", t(x,y,z): " << odomPtr->pose.pose.position.x << "," << odomPtr->pose.pose.position.y << "," << odomPtr->pose.pose.position.z << std::endl;

  //New timestampd and pose
  double ts = odomPtr->header.stamp.toSec();
  auto& p = odomPtr->pose.pose;
  gtsam::Pose3 pose(gtsam::Rot3(p.orientation.w, p.orientation.x, p.orientation.y, p.orientation.z),
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
    auto old_key = _state.key();
    new_key = newStateKey();
    addPoseBetweenFactor(old_key, new_key, pose);
  }
  //Get result - LiDAR frame
  gtsam::Values result = _graph->calculateEstimate();
  _state.updateNavState(new_key, ts, gtsam::NavState(result.at<gtsam::Pose3>(X(new_key)), gtsam::Vector3(0, 0, 0)));
  auto t2 = std::chrono::high_resolution_clock::now();

  //Convert to Alphasense IMU frame
  gtsam::Pose3 lidar_pose = _state.navState().pose();      //w_L
  gtsam::Pose3 imu_pose = lidar_pose.compose(_lidar2imu);  //w_L * B_L = w_B

  //Publish Graph path
  //Pose Message
  static size_t poseMsgCount = 0;
  geometry_msgs::PoseStamped poseMsg;
  poseMsg.header.frame_id = "alphasense_imu";
  poseMsg.header.stamp = odomPtr->header.stamp;
  poseMsg.header.seq = ++poseMsgCount;
  poseMsg.pose.position.x = imu_pose.translation().x();
  poseMsg.pose.position.y = imu_pose.translation().y();
  poseMsg.pose.position.z = imu_pose.translation().z();
  poseMsg.pose.orientation.x = imu_pose.rotation().toQuaternion().x();
  poseMsg.pose.orientation.y = imu_pose.rotation().toQuaternion().y();
  poseMsg.pose.orientation.z = imu_pose.rotation().toQuaternion().z();
  poseMsg.pose.orientation.w = imu_pose.rotation().toQuaternion().w();

  //Node Message
  maplab_msgs::TrajectoryNode nodeMsg;
  nodeMsg.robot_name = "cerberus";
  nodeMsg.pose = std::move(poseMsg);

  //Path Message
  _trajectoryMsg.header.frame_id = "odom";
  _trajectoryMsg.header.stamp = poseMsg.header.stamp;
  _trajectoryMsg.nodes.emplace_back(std::move(nodeMsg));

  //Publish Path
  _pubGraphPath.publish(_trajectoryMsg);

  // DEBUG
  std::cout << "lidar_pose t(x,y,z): " << lidar_pose.translation().transpose() << ", RPY(deg): " << lidar_pose.rotation().rpy().transpose() * (180.0 / M_PI)
            << "\nimu_pose  t(x,y,z): " << imu_pose.translation().transpose() << ", RPY(deg): " << imu_pose.rotation().rpy().transpose() * (180.0 / M_PI)
            << ", time(ms): " << std::chrono::duration_cast<std::chrono::milliseconds>(t2 - t1).count() << std::endl;
  // DEBUG
}

void GraphIntegrator::maplabPathCallback(const nav_msgs::Path::ConstPtr& pathPtr) {
  std::cout << std::fixed << "\033[36mPATH\033[0m - seq:" << pathPtr->header.seq
            << ", ts: " << pathPtr->header.stamp.toSec()
            << ", Poses: " << pathPtr->poses.size() << std::endl;
}

bool GraphIntegrator::addPriorFactor(const gtsam::Key key, const gtsam::Pose3& pose) {
  //Prior factor noise
  auto poseNoise = gtsam::noiseModel::Diagonal::Sigmas((gtsam::Vector(6) << 1e-3, 1e-3, 1e-3, 1e-6, 1e-6, 1e-6).finished());  // rad,rad,rad,m, m, m
  //Create prior factors
  _newFactors.resize(0);
  _newFactors.emplace_shared<gtsam::PriorFactor<gtsam::Pose3>>(X(key), pose, poseNoise);  //POSE - PriorFactor format is (key,value,matrix) value is same type as type of PriorFactor
  //Initial estimate
  gtsam::Values estimate;
  estimate.insert(X(key), pose);
  //Update graph
  _graph->update(_newFactors, estimate);
  _newFactors.resize(0);  //Reset
}

bool GraphIntegrator::addPoseBetweenFactor(const gtsam::Key old_key, const gtsam::Key new_key, const gtsam::Pose3& pose) {
  //Calculate Pose delta
  gtsam::Pose3 pose_delta = _state.navState().pose().between(pose);
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

}  // namespace loam
