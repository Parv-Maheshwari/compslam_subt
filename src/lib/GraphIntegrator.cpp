#include "loam/GraphIntegrator.hpp"

namespace loam {
bool GraphIntegrator::setup(ros::NodeHandle& node, ros::NodeHandle& privateNode) {
  //Subscribers
  _subMapOdom = node.subscribe<nav_msgs::Odometry>("/loam_recorded_odometry", 10, &GraphIntegrator::mapOdomCallback, this);
  _subMaplabPath = node.subscribe<nav_msgs::Path>("/maplab_server/global_sparse_graph", 2, &GraphIntegrator::maplabPathCallback, this);

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
  _graph->params().print("Factor Graph Parameters:");
  std::cout << "Pose Between Factor Noise - RPY(rad): " << _poseNoise[0] << "," << _poseNoise[1] << "," << _poseNoise[2]
            << ", XYZ(m): " << _poseNoise[3] << "," << _poseNoise[4] << "," << _poseNoise[5] << std::endl;

  return true;
}

void GraphIntegrator::mapOdomCallback(const nav_msgs::Odometry::ConstPtr& odomPtr) {
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
  //Get result
  gtsam::Values result = _graph->calculateEstimate();
  _state.updateNavState(new_key, ts, gtsam::NavState(result.at<gtsam::Pose3>(X(new_key)), gtsam::Vector3(0, 0, 0)));
  auto t2 = std::chrono::high_resolution_clock::now();

  // DEBUG
  auto& ns = _state.navState();
  std::cout << "Graph t(x,y,z): " << ns.position().transpose() << ", RPY(deg): " << ns.attitude().rpy().transpose() * (180.0 / M_PI)
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