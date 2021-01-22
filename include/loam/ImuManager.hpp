#ifndef LOAM_IMUBUFFER_H
#define LOAM_IMUBUFFER_H

//C++
#include <chrono>
#include <deque>
#include <memory>
#include <mutex>

//ROS
#include <ros/ros.h>

//Eigen
#include <Eigen/Core>

namespace loam {
class ImuManager {
 public:
  //Constructor
  ImuManager(const size_t maxSize = 200)
      : _maxBufferSize(maxSize) {}

  //Destructor
  ~ImuManager() {}

  /** \brief Reset Max allowed size of IMU Buffer
     *
     * @param newSize number of maximum IMU messages to be kept in the buffer
  */
  inline void resetBufferSize(const size_t& newSize) { _maxBufferSize = newSize; }

  /** \brief Add IMU data to buffer. Remove oldest element if buffer size exceeds _maxBufferSize
     *
     * @param msg IMU data
  */
  void addToBuffer(const std::shared_ptr<Eigen::Vector4d>& msg) {
    {
      std::lock_guard<std::mutex> lock(_imuMutex);
      //Add new message
      _buffer.push_front(msg);
      //Remove oldest message if over max size
      if (_buffer.size() > _maxBufferSize)
        _buffer.pop_back();
    }
  }

  /** \brief Retrieve nearest(before & after) IMU messages from the buffer according to a given timestamp
     *
     * @param ts input timestamp
     * @param befMsg nearest IMU message BEFORE query timestamp
     * @param aftMsg nearest IMU message AFTER query timestamp
     * @return true if interpolation can be performed else extrapolation is required
  */
  bool getNearestIMUMessages(const double ts, Eigen::Vector4d& befMsg, Eigen::Vector4d& aftMsg) {
    //Loop through IMU buffer to find nearest messages
    size_t idx = 0;
    // auto t1 = std::chrono::high_resolution_clock::now();
    {
      std::lock_guard<std::mutex> lock(_imuMutex);
      while (idx < _buffer.size()) {
        //Check timestamps and reterive messages
        if ((*_buffer.at(idx))[0] < ts) {
          if (idx != 0) {  //Interpolation
            aftMsg = *_buffer.at(idx - 1);
            befMsg = *_buffer.at(idx);
          } else {  //Extrapolation is needed hence get last two messages
            aftMsg = *_buffer.at(idx);
            befMsg = *_buffer.at(idx + 1);
          }
          break;
        }
        ++idx;  //Increment index
      }
    }
    // auto t2 = std::chrono::high_resolution_clock::now();

    bool canInterpolate = ((idx == 0) ? false : true);

    //DEBUG
    if (!canInterpolate)
      ROS_WARN("--- Input Query Timestamp is EARLIER than Latest IMU Message ---");
    // std::cout << std::fixed << "\033[36mIMU\033[0m PCL_ts:" << ts
    //           << ", aft_ts/bef_ts: " << aftMsg[0] << "/" << befMsg[0]
    //           << ", diff(s): " << aftMsg[0] - ts << "/" << befMsg[0] - ts
    //           << ", query time(ns): " << std::chrono::duration_cast<std::chrono::nanoseconds>(t2 - t1).count() << std::endl;  //300ns-1us

    return canInterpolate;
  }

  /** \brief Estimate IMU Measurement at timestamp using interpolation or extrapolation
     *
     * @param rosTimestamp input timestamp in ros::Time 
     * @param imuMsg estimated IMU measurement at given timestamp
     * @return true/false if estimated using interpolation/extrapolation
  */
  bool estimateIMUMeasurementAtTimestamp(const ros::Time& rosTimestamp, Eigen::Vector4d& imuMeas) {
    //Convert ROS timestamp
    double ts = rosTimestamp.toSec();

    //Get Nearest IMU Messages
    Eigen::Vector4d beforeMsg, afterMsg;
    bool interpolate = getNearestIMUMessages(ts, beforeMsg, afterMsg);

    //Calculate timestamp difference ratio for interpolation (invert for extrapolation)
    double timeDiffRatio = (ts - beforeMsg[0]) / (afterMsg[0] - beforeMsg[0]);  //(t2-t1)/(t3-t1)

    //Interpolate
    if (interpolate)
      imuMeas = (afterMsg - beforeMsg) * timeDiffRatio + beforeMsg;  //y2 = (y3-y1)*timeDiffRatio + y1
    //Extrapolate
    else
      imuMeas = (afterMsg - beforeMsg) * (1.0 / timeDiffRatio) + beforeMsg;  //y3 = (y2-y1)*(1.0/timeDiffRatio) + y1

    //Append original timestamp
    imuMeas[0] = ts;

    return interpolate;
  }

  /** \brief Estimate Roll-Pitch Quaternion w.r.t gravity using IMU measurements at timestamp
     *
     * @param rosTimestamp input timestamp in ros::Time 
     * @param qG estimated quaternion w.r.t gravity using estimated IMU acceleration measurement at given timestamp - roll and pitch
     * @param estAccData estimated IMU acceleration measurement at given timestamp
     * @return true/false if estimated acceleration measurement was interpolated/extrapolated
  */
  bool getRollPitchfromIMU(const ros::Time& rosTimestamp, Eigen::Quaterniond& qG, Eigen::Vector4d& estAccData) {
    //Get estimated IMU data(acceleration) at timestamp if not first message
    bool interpolateStatus = false;
    if (_rollPitchInit)
      interpolateStatus = estimateIMUMeasurementAtTimestamp(rosTimestamp, estAccData);
    else {
      //Get mean of all buffer values for initialization
      getIMUBufferMean(estAccData);
      _rollPitchInit = true;
    }

    //Set acceleration vector and estimate euler angles
    Eigen::Vector3d a_vec(estAccData[1], estAccData[2], estAccData[3]);
    a_vec.normalize();
    qG.setFromTwoVectors(a_vec, _gVec);
    qG.normalize();

    return interpolateStatus;
  }

  /** \brief Calculates Mean of all IMU measurements in the buffer
     *
     * @param mean mean of IMU measurements in the buffer
  */
  void getIMUBufferMean(Eigen::Vector4d& mean) {
    mean.setZero();
    std::lock_guard<std::mutex> lock(_imuMutex);
    for (auto& bPtr : _buffer)
      mean += *bPtr;
    mean /= _buffer.size();
  }

 private:
  std::mutex _imuMutex;
  bool _rollPitchInit = false;
  size_t _maxBufferSize;
  std::deque<std::shared_ptr<Eigen::Vector4d>> _buffer;
  const Eigen::Vector3d _gVec = Eigen::Vector3d(0.0, 0.0, 1.0);
};
}  // namespace loam
#endif