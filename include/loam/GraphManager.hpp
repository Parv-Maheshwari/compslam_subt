#ifndef GRAPH_MANAGER_HPP_
#define GRAPH_MANAGER_HPP_

//GTSAM
#define SLOW_BUT_CORRECT_BETWEENFACTOR  // increases accuracy in handling rotations
#include <gtsam/inference/Symbol.h>
#include <gtsam/navigation/CombinedImuFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam_unstable/nonlinear/IncrementalFixedLagSmoother.h>

#include "loam/GraphState.hpp"
#include "loam/ImuManager.hpp"

using gtsam::symbol_shorthand::B;  // Bias  (ax,ay,az,gx,gy,gz)
using gtsam::symbol_shorthand::V;  // Vel   (xdot,ydot,zdot)
using gtsam::symbol_shorthand::X;  // Pose3 (R,t)

namespace loam {
class GraphManager {
 public:
  //Constructor / Destructor
  GraphManager(){};
  ~GraphManager(){};

  //
  bool initializeGraph(const double ts, const gtsam::Rot3 init_attitude);

  //
  bool initializeImuPreintegrator(const double g);

  //
  bool updateImuPreintegrator(const IMUMap& imu_meas);

  //
  bool addImuFactor(const gtsam::Key old_key, const gtsam::Key new_key, const IMUMap& imu_meas);

  //
  bool setImuLidarTransforms(const Eigen::Vector3d& t, const Eigen::Quaterniond& q);

  /** \brief Invert a homogenous transformation matrix
 *
 * @param m_in input 4x4 matrix
 * @param m_out inverted 4x4 matrix
 */
  void invertHomogenousMatrix(const Eigen::Matrix4d& m_in, Eigen::Matrix4d& m_out);

  //Associate timestamp to each 'value key', e.g. for graph key 0, value keys (x0,v0,b0) need to be associated
  void valuesToKeyTimeStampMap(const gtsam::Values& values, const double ts, std::map<gtsam::Key, double>& key_timestamp_map) {
    for (const auto& value : values)
      key_timestamp_map[value.key] = ts;
  }

  //Accessors - Setters
  void setAccNoiseDensity(double val) { _accNoiseDensity = val; }
  void setAccBiasRandomWalk(double val) { _accBiasRandomWalk = val; }
  void setAccBiasPrior(double val) { _accBiasPrior = val; }
  void setGyroNoiseDensity(double val) { _gyrNoiseDensity = val; }
  void setGyrBiasRandomWalk(double val) { _gyrBiasRandomWalk = val; }
  void setGyrBiasPrior(double val) { _gyrBiasPrior = val; }
  void setSmootherLag(double val) { _smootherLag = val; }
  void setIterations(int val) { _iter = val; }
  void setPositionReLinTh(double val) { _posReLinTh = val; }
  void setRotationReLinTh(double val) { _rotReLinTh = val; }
  void setVelocityReLinTh(double val) { _velReLinTh = val; }
  void setAccBiasReLinTh(double val) { _accBiasReLinTh = val; }
  void setGyrBiasReLinTh(double val) { _gyrBiasReLinTh = val; }
  //Accessors - Getters
  auto iterations() const { return _iter; }
  const auto newStateKey() { return ++_state_key; }
  const auto& T_LB() const { return _T_LB; }
  const auto& T_BL() const { return _T_BL; }

  //Objects
  boost::shared_ptr<gtsam::PreintegratedCombinedMeasurements::Params> _imuParams;
  std::shared_ptr<gtsam::imuBias::ConstantBias> _imuBiasPrior;
  std::shared_ptr<gtsam::PreintegratedCombinedMeasurements> _imuPreintegrator;
  gtsam::ISAM2Params _params;
  gtsam::NonlinearFactorGraph _factors;
  std::shared_ptr<gtsam::IncrementalFixedLagSmoother> _graph;
  State _state;

 private:
  //Member variables
  // IMU Preintegration
  double _accNoiseDensity;
  double _accBiasRandomWalk;
  double _accBiasPrior;
  double _gyrNoiseDensity;
  double _gyrBiasRandomWalk;
  double _gyrBiasPrior;
  //Factor Graph
  gtsam::Key _state_key;
  double _smootherLag;
  int _iter;
  double _posReLinTh;
  double _rotReLinTh;
  double _velReLinTh;
  double _accBiasReLinTh;
  double _gyrBiasReLinTh;
  //Transforms
  Eigen::Matrix4d _T_LB = Eigen::Matrix4d::Identity();  //IMU to LiDAR expressed in IMU frame, Read as Transforms LiDAR into IMU
  Eigen::Matrix4d _T_BL = Eigen::Matrix4d::Identity();  //LiDAR to IMU expressed in LiDAR frame, Read as Transforms IMU into LiDAR
};
}  //namespace loam

#endif