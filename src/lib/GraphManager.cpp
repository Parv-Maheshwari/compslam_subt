#include "loam/GraphManager.hpp"

namespace loam {

bool GraphManager::initializeGraph(const double ts, const gtsam::Rot3 init_attitude) {
  //Set graph relinearization thresholds - must be lower case letters, check:gtsam::symbol_shorthand
  gtsam::FastMap<char, gtsam::Vector> relinTh;
  relinTh['x'] = (gtsam::Vector(6) << _rotReLinTh, _rotReLinTh, _rotReLinTh, _posReLinTh, _posReLinTh, _posReLinTh).finished();
  relinTh['v'] = (gtsam::Vector(3) << _velReLinTh, _velReLinTh, _velReLinTh).finished();
  relinTh['b'] = (gtsam::Vector(6) << _accBiasReLinTh, _accBiasReLinTh, _accBiasReLinTh, _gyrBiasReLinTh, _gyrBiasReLinTh, _gyrBiasReLinTh).finished();
  _params.relinearizeThreshold = relinTh;

  //Create Prior factor and Initialize factor graph
  //Prior factor noise
  auto poseNoise = gtsam::noiseModel::Diagonal::Sigmas((gtsam::Vector(6) << 0.01, 0.01, 0.01, 0.0001, 0.0001, 0.0001).finished());  // rad,rad,rad,m, m, m
  auto velocityNoise = gtsam::noiseModel::Isotropic::Sigma(3, 0.0001);                                                              // m/s
  double accSigma = _accBiasRandomWalk * _accBiasRandomWalk;
  double gyrSigma = _gyrBiasRandomWalk * _gyrBiasRandomWalk;
  auto biasNoise = gtsam::noiseModel::Diagonal::Sigmas((gtsam::Vector(6) << accSigma, accSigma, accSigma, gyrSigma, gyrSigma, gyrSigma).finished());
  //Create prior factors
  _state_key = 0;
  gtsam::Pose3 init_pose(init_attitude, gtsam::Point3());
  _factors.resize(0);
  _factors.emplace_shared<gtsam::PriorFactor<gtsam::Pose3>>(X(_state_key), init_pose, poseNoise);                       //POSE - PriorFactor format is (key,value,matrix) value is same type as type of PriorFactor
  _factors.emplace_shared<gtsam::PriorFactor<gtsam::Vector3>>(V(_state_key), gtsam::Vector3(0, 0, 0), velocityNoise);   //VELOCITY
  _factors.emplace_shared<gtsam::PriorFactor<gtsam::imuBias::ConstantBias>>(B(_state_key), *_imuBiasPrior, biasNoise);  //BIAS
  //Initial estimate
  gtsam::Values estimate;
  estimate.insert(X(_state_key), init_pose);
  estimate.insert(V(_state_key), gtsam::Vector3(0, 0, 0));
  estimate.insert(B(_state_key), *_imuBiasPrior);

  //Initialize factor graph
  _graph = std::make_shared<gtsam::IncrementalFixedLagSmoother>(_smootherLag, _params);
  _params.print("Factor Graph Parameters:");

  //Add prior factor to graph and update
  std::map<gtsam::Key, double> keyTimestampMap;
  valuesToKeyTimeStampMap(estimate, ts, keyTimestampMap);
  _graph->update(_factors, estimate, keyTimestampMap);

  //Reset
  _factors.resize(0);

  //Update Current State
  _state.updateNavStateAndBias(_state_key,
                               ts,
                               gtsam::NavState(init_pose, gtsam::Vector3(0, 0, 0)),
                               *_imuBiasPrior);

  //DEBUG
  gtsam::Values result = _graph->calculateEstimate();
  result.print("Prior Factor Result:");
}

bool GraphManager::initializeImuPreintegrator(const double g) {
  //Initialize IMU Preintegrator
  _imuParams = gtsam::PreintegratedCombinedMeasurements::Params::MakeSharedU(g);
  _imuParams->accelerometerCovariance = gtsam::Matrix33::Identity(3, 3) * _accNoiseDensity;
  _imuParams->biasAccCovariance = gtsam::Matrix33::Identity(3, 3) * _accBiasRandomWalk;
  _imuParams->gyroscopeCovariance = gtsam::Matrix33::Identity(3, 3) * _gyrNoiseDensity;
  _imuParams->biasOmegaCovariance = gtsam::Matrix33::Identity(3, 3) * _gyrBiasRandomWalk;
  _imuParams->integrationCovariance = gtsam::Matrix33::Identity(3, 3) * 1.0e-8;
  // _imuParams->biasAccOmegaInt = gtsam::Matrix66::Identity(6, 6) * 1.0e-5;
  gtsam::Vector3 acc_bias_prior(_accBiasPrior, _accBiasPrior, _accBiasPrior);
  gtsam::Vector3 gyr_bias_prior(_gyrBiasPrior, _gyrBiasPrior, _gyrBiasPrior);
  _imuBiasPrior = std::make_shared<gtsam::imuBias::ConstantBias>(acc_bias_prior, gyr_bias_prior);
  _imuPreintegrator = std::make_shared<gtsam::PreintegratedCombinedMeasurements>(_imuParams, *_imuBiasPrior);
  _imuParams->print("IMU Preintegration Parameters:");
  return true;
}

bool GraphManager::updateImuPreintegrator(const IMUMap& imu_meas) {
  if (imu_meas.size() < 2) {
    std::cout << "updateImuPreintegrator --- Received less than 2 IMU messages --- No Preintegration done" << std::endl;
    return false;
  }

  //Reset IMU Pre-integration
  _imuPreintegrator->resetIntegrationAndSetBias(_state.imuBias());

  //Start integrating with imu_meas.begin()+1 meas to calculate dt //NOTE imu_meas.begin() meas was integrated before
  auto curr_itr = imu_meas.begin();
  auto prev_itr = curr_itr;
  ++curr_itr;

  //Calculate dt and integrate IMU measurements
  size_t count = 0;
  for (; curr_itr != imu_meas.end(); ++curr_itr, ++prev_itr) {
    double dt = curr_itr->first - prev_itr->first;
    _imuPreintegrator->integrateMeasurement(curr_itr->second.head<3>(),  //acc
                                            curr_itr->second.tail<3>(),  //gyro
                                            dt);                         //delta t
    ++count;
  }

  //DEBUG
  if (true /*kDisplayDebugText_*/)
    std::cout << std::fixed << "updateImuPreintegrator -- # of IMU msgs: " << count
              << ", with Start/End ts: " << imu_meas.begin()->first << "/" << imu_meas.rbegin()->first
              << ", diff: " << imu_meas.rbegin()->first - imu_meas.begin()->first << std::endl;
  //DEBUG

  return true;
}

bool GraphManager::addImuFactor(const gtsam::Key old_key, const gtsam::Key new_key, const IMUMap& imu_meas) {
  //Update IMU preintegrator
  bool success = updateImuPreintegrator(imu_meas);
  if (!success)
    return success;

  //Create IMU Factor
  gtsam::CombinedImuFactor imu_factor(X(old_key), V(old_key),
                                      X(new_key), V(new_key),
                                      B(old_key), B(new_key),
                                      *_imuPreintegrator);

  //Add factor
  _factors.add(imu_factor);

  return success;
}

bool GraphManager::setImuLidarTransforms(const Eigen::Vector3d& t, const Eigen::Quaterniond& q) {
  //IMU to LiDAR expressed in IMU frame
  _T_LB.block<3, 1>(0, 3) = t;
  _T_LB.block<3, 3>(0, 0) = q.toRotationMatrix();
  //LiDAR to IMU expressed in LiDAR frame
  invertHomogenousMatrix(_T_LB, _T_BL);
  std::cout << "IMU-to-LiDAR Extrinsic Calibration:\n"
            << _T_LB << std::endl;
  std::cout << "LiDAR-to-IMU Extrinsic Calibration:\n"
            << _T_BL << std::endl;
}

void GraphManager::invertHomogenousMatrix(const Eigen::Matrix4d& m_in, Eigen::Matrix4d& m_out) {
  Eigen::Matrix3d R = m_in.block<3, 3>(0, 0);
  Eigen::Vector3d t = m_in.block<3, 1>(0, 3);
  m_out.block<3, 3>(0, 0) = R.transpose();
  m_out.block<3, 1>(0, 3) = -R.transpose() * t;
}

}  //namespace loam