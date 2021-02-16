#ifndef GRAPH_INTEGRATOR_HPP
#define GRAPH_INTEGRATOR_HPP

//ROS
#include <eigen_conversions/eigen_msg.h>
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
class GraphIntegrator {
 public:
  //Constructor / Destructor
  GraphIntegrator(){};
  ~GraphIntegrator(){};

  //Setup - Read Params
  bool setup(ros::NodeHandle& node, ros::NodeHandle& privateNode);

 private:
  //Fucntions
  //Callbacks
  void laserMapOdomCallback(const nav_msgs::Odometry::ConstPtr& odomPtr);
  void maplabPathCallback(const nav_msgs::Path::ConstPtr& pathPtr);
  const auto newStateKey() { return ++_state_key; }
  //Add prior
  bool addPriorFactor(const gtsam::Key key, const gtsam::Pose3& pose);
  //Add a pose between factor
  bool addPoseBetweenFactor(const gtsam::Key old_key, const gtsam::Key new_key, const gtsam::Pose3& pose);

  //Members
  ros::Subscriber _subLaserMapOdom;  // Laser mapping odometry subscriber
  ros::Subscriber _subMaplabPath;    // Maplab published path subscriber
  ros::Publisher _pubGraphPath;      // Graph path publisher
  nav_msgs::Path _pathMsg;           // ROS path message for graph

  //Transforms
  gtsam::Pose3 _lidar2imu;

  //Factor graph
  gtsam::ISAM2Params _params;
  gtsam::NonlinearFactorGraph _newFactors;
  std::shared_ptr<gtsam::ISAM2> _graph;
  bool _isGraphInitialized = false;
  State _state;
  gtsam::Key _state_key = 0;  //Current state key
  //Pose Between Factor
  std::vector<double> _poseNoise{0.02, 0.02, 0.02, 0.05, 0.05, 0.05};  //ORDER RPY(rad) - XYZ(meters)
};
}  // namespace loam

#endif