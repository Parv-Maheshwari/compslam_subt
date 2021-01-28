#ifndef GRAPH_MANAGER_HPP_
#define GRAPH_MANAGER_HPP_

//GTSAM
#define SLOW_BUT_CORRECT_BETWEENFACTOR  // increases accuracy in handling rotations
#include <gtsam/inference/Symbol.h>
#include <gtsam/navigation/CombinedImuFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam_unstable/nonlinear/IncrementalFixedLagSmoother.h>

using gtsam::symbol_shorthand::X;  // Pose3 (R,t)
using gtsam::symbol_shorthand::V;  // Vel   (xdot,ydot,zdot)
using gtsam::symbol_shorthand::B;  // Bias  (ax,ay,az,gx,gy,gz)

namespace loam {
class GraphManager {
 public:
  //Constructor / Destructor
  GraphManager(){};
  ~GraphManager(){};

  //Accessors
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

  auto smootherLag() const { return _smootherLag; }
  auto iterations() const { return _iter; }
  auto posReLinTh() const { return _posReLinTh; }
  auto rotReLinTh() const { return _rotReLinTh; }
  auto velReLinTh() const { return _velReLinTh; }
  auto accBiasReLinTh() const { return _accBiasReLinTh; }
  auto gyrBiasReLinTh() const { return _gyrBiasReLinTh; }

  auto accNoiseDensity() const { return _accNoiseDensity; }
  auto accBiasRandomWalk() const { return _accBiasRandomWalk; }
  auto accBiasPrior() const { return _accBiasPrior; }
  auto gyrNoiseDensity() const { return _gyrNoiseDensity; }
  auto gyrBiasRandomWalk() const { return _gyrBiasRandomWalk; }
  auto gyrBiasPrior() const { return _gyrBiasPrior; }

  //Objects
  boost::shared_ptr<gtsam::PreintegratedCombinedMeasurements::Params> _imuParams;
  std::shared_ptr<gtsam::imuBias::ConstantBias> _imuBiasPrior;
  std::shared_ptr<gtsam::PreintegratedCombinedMeasurements> _imuPreintegrator;
  gtsam::ISAM2Params _params;
  gtsam::NonlinearFactorGraph _factors;
  std::shared_ptr<gtsam::IncrementalFixedLagSmoother> _graph;

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
  double _smootherLag;
  int _iter;
  double _posReLinTh;
  double _rotReLinTh;
  double _velReLinTh;
  double _accBiasReLinTh;
  double _gyrBiasReLinTh;
  
};
}  //namespace loam

#endif