#ifndef GRAPH_INTEGRATOR_HPP
#define GRAPH_INTEGRATOR_HPP

//C++
#include <algorithm>
#include <unordered_map>
#include <vector>

//ROS
#include <eigen_conversions/eigen_msg.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/synchronizer.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <pcl/io/pcd_io.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>

//PCL
#include <pcl/common/transforms.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

//GTSAM
#define SLOW_BUT_CORRECT_BETWEENFACTOR  // increases accuracy in handling rotations
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/PriorFactor.h>

//CUSTOM
#include "loam/GraphState.hpp"

using gtsam::symbol_shorthand::X;  // Pose3 (R,t)

namespace loam {
class MaplabIntegrator {
 public:
  //Setup
  bool setup(ros::NodeHandle& node, ros::NodeHandle& privateNode);

 private:
  //Callbacks
  void syncCallbackHandler(const nav_msgs::OdometryConstPtr& OdomPtr, const sensor_msgs::PointCloud2ConstPtr& cloudPtr);
  int lidarOdomCallback(const nav_msgs::Odometry::ConstPtr& odomPtr);
  void cloudCallback(int key, const sensor_msgs::PointCloud2ConstPtr& cloudPtr);
  void absolutePoseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& posePtr);
  void maplabAnchorCallback(const nav_msgs::Path::ConstPtr& pathPtr);
  void maplabSubmapCallback(const nav_msgs::Path::ConstPtr& pathPtr);

  //State Key increment
  const auto newStateKey() { return ++_state_key; }
  //Add prior
  void addPriorFactor(const gtsam::Key key, const gtsam::Pose3& pose);
  //Add a pose between factor
  void addPoseBetweenFactor(const gtsam::Key old_key, const gtsam::Key new_key, const gtsam::Pose3& pose_delta, const gtsam::Pose3& pose_estimate);
  //Add prior factor from global graph
  bool findClosestKeyForTS(const double ts, gtsam::Key* key) const; //TODO - deprecate
  //Visualize the ISAM graph
  void visualizeResults();
  //Publish associated transforms
  void publishTransforms(const ros::Time& ts);

  //Utility methods
  bool createPoseMessage(const gtsam::Pose3& pose, geometry_msgs::PoseStamped* pose_msg) const;
  void updateResultsForVisualization(const ros::Time ts);

  //Lookup maps for key-factor association
  void updateKeyAnchorFactorIdxMap(const gtsam::Key key) { _keyAnchorFactorIdxMap[key] = _factor_count - 1; }
  void updateKeyAnchorPoseMap(const gtsam::Key key, const gtsam::Pose3& pose) { _keyAnchorPoseMap[key] = pose; }
  void updateKeySubmapFactorIdxMap(const gtsam::Key parent_key, const gtsam::Key child_key);
  int findSubmapFactorIdx(const gtsam::Key parent_key, const gtsam::Key child_key, bool erase = false);

  //Factor count increment and retreivel
  void incFactorCount() { ++_factor_count; }
  size_t getFactorCount() { return _factor_count; }

  //Members

  //Subscribers
  ros::Subscriber _subAbsolutePose;  // Absolute pose constraints from Apriltags
  ros::Subscriber _subMaplabAnchor;  // Maplab published Anchor constraints
  ros::Subscriber _subMaplabSubmap;  // Maplab published Submap-to-Submap contraints
  //Synced subscriber
  message_filters::Subscriber<nav_msgs::Odometry> _subOdom;
  message_filters::Subscriber<sensor_msgs::PointCloud2> _subCloud;
  typedef message_filters::sync_policies::ExactTime<nav_msgs::Odometry, sensor_msgs::PointCloud2> _syncPolicy;  // ROS Sync Policy Object
  boost::shared_ptr<message_filters::Synchronizer<_syncPolicy>> _syncPtr;                                       // ROS  Sync Policy Message Filter

  //Publishers
  ros::Publisher _pubGraphPath;    // Current state (Incremental) of graph
  ros::Publisher _pubUpdatedPath;  // Optimized graph path
  nav_msgs::Path _pathMsg;         // ROS path message for graph
  ros::Publisher _pubMap;          // Publish corrected Map

  //Frame names and transforms
  std::string _world_frame = "world";
  std::string _map_frame = "map";
  std::string _base_frame = "imu";
  tf::TransformBroadcaster _tb;              // transform broadcaster
  geometry_msgs::TransformStamped _optPose;  // Optimized Pose
  geometry_msgs::TransformStamped _incPose;  // incremental Pose

  //Transforms
  gtsam::Pose3 _T_B_L;  //LiDAR(L) to alphasense IMU(B)
  gtsam::Pose3 _T_B_C;  //alphasense camera(C) to alphasense IMU(B)
  gtsam::Pose3 _T_G_B;  //alphasense IMU(B) to DARPA(G)
  gtsam::Pose3 _T_G_M;  //DARPA(G) to Robot (Local) Map(M) - Local Robot Map start at origin MUST BE SET TO ZERO ON INIT in (B) frame - this needs to be updated to make the local graph expressed w.r.t (G)

  //Factor graph
  size_t _factor_count = 0;               //Counter for Total factors (existing + removed)
  std::mutex _graphMutex;                   //For adding new factors and graph update
  gtsam::ISAM2Params _params;               //Graph parameters
  gtsam::NonlinearFactorGraph _newFactors;  //New factors to be added to the graph
  std::shared_ptr<gtsam::ISAM2> _graph;     //iSAM2 GRAPH object
  gtsam::Key _state_key = 0;                //Current state key
  std::vector<StatePtr> _states;            //Vecotr of states //TODO deprecate
  
  //Factor noise vectors - ORDER RPY(rad) - XYZ(meters)
  gtsam::Vector6 _odomNoise;      //Odometry BetweenFactor Noise
  gtsam::Vector6 _absoluteNoise;  //Absolute(AprilTag) PriorFactor Noise
  gtsam::Vector6 _submapNoise;    //Submap BetweenFactor Noise
  gtsam::Vector6 _anchorNoise;    //Anchor PriorFactor Noise

  //Odometry factor
  bool _firstOdomMsg = true;
  gtsam::Pose3 _lastIMUPose;

  //Absolute pose factor
  bool _firstAbsolutePose = true;

  //Lookup map objects for key-to-factorIndex associations
  std::map<double, gtsam::Key> _timestampKeyMap;                            //Timestamp-Key map for lookup of keys corresponding to odometry timestamps
  std::map<gtsam::Key, size_t> _keyAnchorFactorIdxMap;                      //Key-PriorFactorIndex map for lookup of indices of prior factor add at key for Anchor poses
  std::unordered_map<gtsam::Key, gtsam::Pose3> _keyAnchorPoseMap;           //Key-AnchorPose map for lookup of applied anchor pose as prior factor at Key
  std::unordered_map<gtsam::Key, std::set<size_t>> _keySubmapFactorIdxMap;  //Key-SubmapBetweenFactorIndex map for lookup of indices of betweenfactor added at key for Submap constraints

  //Timer-based map/result publisher
  bool _new_results = false;
  gtsam::Values _results;
  ros::Time _resultTimestamp;
  ros::Timer _timerVisualizeResults;

  //Pointcloud
  double _cloudSavePosDelta = 1.0;                                                   //minimum delta position difference Norm to save new pointcoud (meters)
  double _cloudSaveRotDelta = 0.2;                                                   //minimum delta rotation difference Norm to save new pointcoud (radians)
  gtsam::Pose3 _lastCloudSavePose;                                                   //pose of last pointcloud saved
  std::unordered_map<gtsam::Key, pcl::PointCloud<pcl::PointXYZ>::Ptr> _keyCloudMap;  //Key-Cloud map for lookup of pointclouds at keys to build map
};

}  // namespace loam

#endif