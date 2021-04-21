#ifndef GRAPH_INTEGRATOR_HPP
#define GRAPH_INTEGRATOR_HPP

//C++
#include <vector>
#include <unordered_map>

//ROS
#include <eigen_conversions/eigen_msg.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>

//GTSAM
#define SLOW_BUT_CORRECT_BETWEENFACTOR  // increases accuracy in handling rotations
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/PriorFactor.h>

#include "loam/GraphState.hpp"

using gtsam::symbol_shorthand::X;  // Pose3 (R,t)

namespace loam {
class MaplabIntegrator {
 public:
  //Setup - Read Params
  bool setup(ros::NodeHandle& node, ros::NodeHandle& privateNode);

 private:
  //Callbacks
  void lidarOdomCallback(const nav_msgs::Odometry::ConstPtr& odomPtr);
  void absolutePoseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& posePtr);
  void maplabAnchorCallback(const nav_msgs::Path::ConstPtr& pathPtr);
  void maplabSubmapCallback(const nav_msgs::Path::ConstPtr& pathPtr);  

  const auto newStateKey() { return ++_state_key; }
  //Add prior
  void addPriorFactor(const gtsam::Key key, const gtsam::Pose3& pose, const bool addEstimate = true);
  //Add a pose between factor
  void addPoseBetweenFactor(const gtsam::Key old_key, const gtsam::Key new_key, const gtsam::Pose3& pose_delta, const gtsam::Pose3& pose_estimate);
  //Add prior factor from global graph
  bool findClosestKeyForTS(const double ts, gtsam::Key* key) const;
  //Visualize the ISAM graph
  void visualizeOnboardGraph() const;

  //Utility methods
  bool createPoseMessage(const gtsam::Pose3& pose, geometry_msgs::PoseStamped* pose_msg) const;

  //Update 
  void updateKeyAnchorFactorIdxMap(const gtsam::Key key) { _keyAnchorFactorIdxMap[key] = _graph->getFactorsUnsafe().size()-1; }
  void printKeyFactorIdxMap();
  void updateKeyAnchorPoseMap(const gtsam::Key key, const gtsam::Pose3& pose) { _keyAnchorPoseMap[key] = pose; }

  
  void updateKeySubmapFactorIdxMap(const gtsam::Key parent_key, const gtsam::Key child_key);
  int findSubmapFactorIdx(const gtsam::Key parent_key, const gtsam::Key child_key, bool erase = false);

  //Members
  ros::Subscriber _subLaserMapOdom;  // Laser mapping odometry subscriber
  ros::Subscriber _subAbsolutePose;  // Absolute pose constraints from Apriltags
  ros::Subscriber _subMaplabAnchor;  // Maplab published Anchor constraints
  ros::Subscriber _subMaplabSubmap;  // Maplab published Submap-to-Submap contraints

  ros::Publisher _pubGraphPath;      // Graph path publisher
  ros::Publisher _pubUpdatedPath;    //
  nav_msgs::Path _pathMsg;           // ROS path message for graph

  //Transforms
  gtsam::Pose3 _T_B_L; //LiDAR(L) to alphasense IMU(B)
  gtsam::Pose3 _T_B_C; //alphasense camera(C) to alphasense IMU(B)
  gtsam::Pose3 _T_G_B; //alphasense IMU(B) to DARPA(G)
  gtsam::Pose3 _T_G_M; //DARPA(G) to Robot (Local) Map(M) - Local Robot Map start at origin MUST BE SET TO ZERO ON INIT in (B) frame - this needs to be updated to make the local graph expressed w.r.t (G)

  //Factor graph
  std::mutex _graphMutex;                   //For adding new factors and graph update
  gtsam::ISAM2Params _params;
  gtsam::NonlinearFactorGraph _newFactors;
  std::shared_ptr<gtsam::ISAM2> _graph;
  bool _isGraphInitialized = false;
  std::vector<StatePtr> _states;
  gtsam::Key _state_key = 0;  //Current state key
  //Pose Between Factor
  std::vector<double> _poseNoise{0.02, 0.02, 0.02, 0.05, 0.05, 0.05};  //ORDER RPY(rad) - XYZ(meters)
  //LIDAR odometry
  bool _firstLidarPose = true;
  gtsam::Pose3 _lastLidarPose;
  //Absolute poses
  bool _firstAbsolutePose = true;
  
  //Lookup map objects
  std::map<double, gtsam::Key> _timestampKeyMap;                   //Timestamp-Key map for lookup of keys corresponding to odometry timestamps
  std::map<gtsam::Key, size_t> _keyAnchorFactorIdxMap;             //Key-PriorFactorIndex map for lookup of indices of prior factor add at key for Anchor poses
  std::unordered_map<gtsam::Key, gtsam::Pose3> _keyAnchorPoseMap;  //Key-AnchorPose map for lookup of applied anchor pose as prior factor at Key

  std::unordered_map<gtsam::Key, std::set<size_t>> _keySubmapFactorIdxMap;  //Key-SubmapBetweenFactorIndex map for lookup of indices of betweenfactor added at key for Submap constraints
};

}  // namespace loam

#endif