/*
* Copyright (c) 2014, Autonomous Systems Lab
* All rights reserved.
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are met:
* * Redistributions of source code must retain the above copyright
* notice, this list of conditions and the following disclaimer.
* * Redistributions in binary form must reproduce the above copyright
* notice, this list of conditions and the following disclaimer in the
* documentation and/or other materials provided with the distribution.
* * Neither the name of the Autonomous Systems Lab, ETH Zurich nor the
* names of its contributors may be used to endorse or promote products
* derived from this software without specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
* ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
* WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
* DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
* FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
* DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
* SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
* CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
* OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
* OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*
*/

#ifndef ROVIO_IMGUPDATE_HPP_
#define ROVIO_IMGUPDATE_HPP_

#include "lightweight_filtering/State.hpp"
#include "lightweight_filtering/Update.hpp"
#include "lightweight_filtering/common.hpp"
#include "compslam_vtio/Camera.hpp"
#include "compslam_vtio/CoordinateTransform/FeatureOutput.hpp"
#include "compslam_vtio/CoordinateTransform/PixelOutput.hpp"
#include "compslam_vtio/FilterStates.hpp"
#include "compslam_vtio/MultilevelPatchAlignment.hpp"
#include "compslam_vtio/ZeroVelocityUpdate.hpp"

//CUSTOMIZATION
#include <pcl/common/io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <ros/ros.h>
#include <std_msgs/Header.h>
#include <tf/tf.h>
//CUSTOMIZATION

namespace rovio {

/** \brief Class, defining the innovation.
 *
 *  @tparam STATE - Filter State
 */
template <typename STATE>
class ImgInnovation : public LWF::State<LWF::VectorElement<2>> {
 public:
  typedef LWF::State<LWF::VectorElement<2>> Base;
  using Base::E_;
  static constexpr unsigned int _pix = 0;
  ImgInnovation() {
    static_assert(_pix + 1 == E_, "Error with indices");
    this->template getName<_pix>() = "pix";
  };
  virtual ~ImgInnovation(){};
};

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

/** \brief Class, holding the image pyramids of the different cameras.
 *
 *  @tparam STATE - Filter State
 */
template <typename STATE>
class ImgUpdateMeasAuxiliary : public LWF::AuxiliaryBase<ImgUpdateMeasAuxiliary<STATE>> {
 public:
  ImgUpdateMeasAuxiliary() {
    reset(0.0);
  };
  virtual ~ImgUpdateMeasAuxiliary(){};
  void reset(const double t) {
    imgTime_ = t;
    for (int i = 0; i < STATE::nCam_; i++) {
      isValidPyr_[i] = false;
    }
  }
  bool areAllValid() {
    for (int i = 0; i < STATE::nCam_; i++) {
      if (isValidPyr_[i] == false) return false;
    }
    return true;
  }
  ImagePyramid<STATE::nLevels_> pyr_[STATE::nCam_];
  bool isValidPyr_[STATE::nCam_];
  double imgTime_;

  //CUSTOMIZATION
  std_msgs::Header imgheader_, pclHeader_;
  tf::Transform pcl2imgTF_;
  pcl::PointCloud<pcl::PointXYZI> pclCloud_;
  void setImgHeader(const std_msgs::Header& h) { imgheader_ = h; }
  void setCloudHeader(const std_msgs::Header& h) { pclHeader_ = h; }
  void setLidarCameraCalibration(const tf::Transform& lidar2cam) {
    pcl2imgTF_ = lidar2cam;
    std::cout << "--- ROVIO: PCL_to_Image transform, t(x,y,z):" << pcl2imgTF_.getOrigin().x() << "," << pcl2imgTF_.getOrigin().y() << "," << pcl2imgTF_.getOrigin().z()
              << ", q(w,x,y,z):" << pcl2imgTF_.getRotation().w() << pcl2imgTF_.getRotation().x() << pcl2imgTF_.getRotation().y() << pcl2imgTF_.getRotation().z()
              << std::endl;
  }
  pcl::PointCloud<pcl::PointXYZI>& pclCloud() { return pclCloud_; }
  //CUSTOMIZATION
};

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

/**  \brief Update measurement class (all data in auxillary)
 *
 *  @tparam STATE - Filter State
 */
template <typename STATE>
class ImgUpdateMeas : public LWF::State<ImgUpdateMeasAuxiliary<STATE>> {
 public:
  typedef LWF::State<ImgUpdateMeasAuxiliary<STATE>> Base;
  using Base::E_;
  static constexpr unsigned int _aux = 0;
  ImgUpdateMeas() {
    static_assert(_aux + 1 == E_, "Error with indices");
  };
  virtual ~ImgUpdateMeas(){};

  //@{
  /** \brief Get the auxiliary state of the ImgUpdateMeas.
   *
   *  \see ImgUpdateMeasAuxiliary
   *  @return the the auxiliary state of the ImgUpdateMeas.
   */
  inline ImgUpdateMeasAuxiliary<STATE>& aux() {
    return this->template get<_aux>();
  }
  inline const ImgUpdateMeasAuxiliary<STATE>& aux() const {
    return this->template get<_aux>();
  }
  //@}
};

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

/**  \brief Class holding the update noise.
 *
 *  @tparam STATE - Filter State
 */
template <typename STATE>
class ImgUpdateNoise : public LWF::State<LWF::VectorElement<2>> {
 public:
  typedef LWF::State<LWF::VectorElement<2>> Base;
  using Base::E_;
  static constexpr unsigned int _pix = 0;
  ImgUpdateNoise() {
    static_assert(_pix + 1 == E_, "Error with indices");
    this->template getName<_pix>() = "pix";
  };
  virtual ~ImgUpdateNoise(){};
};

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

/** \brief Outlier Detection.
 *
 *  @tparam STATE - Filter State
 */
template <typename STATE>
class ImgOutlierDetection : public LWF::OutlierDetection<LWF::ODEntry<ImgInnovation<STATE>::template getId<ImgInnovation<STATE>::_pix>(), 2>> {
 public:
  /** \brief Destructor
   */
  virtual ~ImgOutlierDetection(){};
};

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

/** \brief Class, holding image update routines for the filter.
 */
template <typename FILTERSTATE>
class ImgUpdate : public LWF::Update<ImgInnovation<typename FILTERSTATE::mtState>,
                                     FILTERSTATE,
                                     ImgUpdateMeas<typename FILTERSTATE::mtState>,
                                     ImgUpdateNoise<typename FILTERSTATE::mtState>,
                                     ImgOutlierDetection<typename FILTERSTATE::mtState>,
                                     false> {
 public:
  typedef LWF::Update<ImgInnovation<typename FILTERSTATE::mtState>,
                      FILTERSTATE,
                      ImgUpdateMeas<typename FILTERSTATE::mtState>,
                      ImgUpdateNoise<typename FILTERSTATE::mtState>,
                      ImgOutlierDetection<typename FILTERSTATE::mtState>,
                      false>
      Base;
  using Base::boolRegister_;
  using Base::cancelIteration_;
  using Base::candidateCounter_;
  using Base::doubleRegister_;
  using Base::hasConverged_;
  using Base::intRegister_;
  using Base::meas_;
  using Base::successfulUpdate_;
  using Base::updnoiP_;
  using Base::useImprovedJacobian_;
  typedef typename Base::mtState mtState;
  typedef typename Base::mtFilterState mtFilterState;
  typedef typename Base::mtInnovation mtInnovation;
  typedef typename Base::mtMeas mtMeas;
  typedef typename Base::mtNoise mtNoise;
  typedef typename Base::mtOutlierDetection mtOutlierDetection;

  // Multicamera pointer
  MultiCamera<mtState::nCam_>* mpMultiCamera_;

  // Parameter
  M3D initCovFeature_;
  double initDepth_;
  int startLevel_;
  int endLevel_;
  double startDetectionTh_;
  int nDetectionBuckets_;
  int fastDetectionThreshold_;
  double scoreDetectionExponent_;
  double penaltyDistance_;
  double zeroDistancePenalty_;
  double trackingUpperBound_, trackingLowerBound_;
  double minTrackedAndFreeFeatures_;
  double minRelativeSTScore_;
  double minAbsoluteSTScore_;
  double minTimeBetweenPatchUpdate_;
  bool doVisualMotionDetection_;            /**<Do visual motion detection*/
  double rateOfMovingFeaturesTh_;           /**<What percentage of feature must be moving for image motion detection*/
  double pixelCoordinateMotionTh_;          /**<Threshold for detecting feature motion*/
  int minFeatureCountForNoMotionDetection_; /**<Minimum amount of feature for detecting NO image motion*/
  double removalFactor_;                    /**<Factor for enforcing feature removal if not enough free*/
  double patchRejectionTh_;
  bool useDirectMethod_; /**<If true, the innovation term is based directly on pixel intensity errors.
                              If false, the reprojection error is used for the innovation term.*/
  bool doFrameVisualisation_;
  bool visualizePatches_;
  bool verbose_;
  bool removeNegativeFeatureAfterUpdate_;
  bool isZeroVelocityUpdateEnabled_;                   /**<Should zero velocity updates be performed*/
  double minTimeForZeroVelocityUpdate_;                /**<Time until zero velocity update get performed if there is no motion*/
  ZeroVelocityUpdate<FILTERSTATE> zeroVelocityUpdate_; /**<Zero velocity update, directly integrated into the img update*/
  double maxUncertaintyToDepthRatioForDepthInitialization_;
  double updateNoisePix_;        /**<Update noise of update pixel, used for indirect case (reprojection error)*/
  double updateNoiseInt_;        /**<Update noise of intensity error, used for direct approach*/
  double noiseGainForOffCamera_; /**<Factor added on update noise if not main camera*/
  double alignConvergencePixelRange_;
  double alignCoverageRatio_;
  int alignMaxUniSample_;
  bool useCrossCameraMeasurements_;        /**<Should features be matched across cameras.*/
  bool doStereoInitialization_;            /**<Should a stereo match be used for feature initialization.*/
  double alignmentHuberNormThreshold_;     /**<Intensity error threshold for Huber norm.*/
  double alignmentGaussianWeightingSigma_; /**<Width of Gaussian which is used for pixel error weighting.*/
  double alignmentGradientExponent_;       /**<Exponent used for gradient based weighting of residuals.*/
  double discriminativeSamplingDistance_;  /**<Sampling distance for checking discriminativity of patch (if <= 0.0 no check is performed).*/
  double discriminativeSamplingGain_;      /**<Gain for threshold above which the samples must lie (if <= 1.0 the patchRejectionTh is used).*/
  double maxAllowedFeatureDistance_;       /**<CUSTOMIZATION: Max allowed estimated feature distance after performing update, remove features with very far away depth and clustered features during backward motion.*/
  double minTimeBeforeFeaDistCheck_;       /**<CUSTOMIZATION: Min time allowed in secd before maxAllowedFeatureDistance check is applied. Typically 1.0sec for camera at 20Hz i.e. 20 updates have been applied*/
  bool useDepthFromLiDAR_;                 /**<CUSTOMIZATION: Enable or Disable usage of LiDAR direct depth for features */
  int lidarDepthCalcMode_;                 /**<CUSTOMIZATION: Use Patch(0) or Line(1) for Depth Calculation */
  int depthSearchWindowWidth_;             /**<CUSTOMIZATION: Horizontal size of direct depth search window*/
  int depthSearchWindowHeight_;            /**<CUSTOMIZATION: Vertical size of direct depth search window*/
  double directDepInitCovScaleFactor_;     /**<CUSTOMIZATION: Factor to scale depth initialization covaraince in case of direct depth initialization*/

  // Temporary
  mutable PixelOutputCT pixelOutputCT_;
  mutable PixelOutput pixelOutput_;
  mutable MXD pixelOutputCov_;
  mutable rovio::TransformFeatureOutputCT<mtState> transformFeatureOutputCT_;
  mutable FeatureOutput featureOutput_;
  mutable MXD featureOutputCov_;
  mutable MXD featureOutputJac_;
  mutable MultilevelPatch<mtState::nLevels_, mtState::patchSize_> mlpTemp1_;
  mutable MultilevelPatch<mtState::nLevels_, mtState::patchSize_> mlpTemp2_;
  mutable FeatureCoordinates alignedCoordinates_;
  mutable FeatureCoordinates tempCoordinates_;
  mutable FeatureCoordinatesVec candidates_;
  mutable cv::Point2f c_temp_;
  mutable Eigen::Matrix2d c_J_;
  mutable Eigen::Matrix2d A_red_;
  mutable Eigen::Vector2d b_red_;

  mutable Eigen::MatrixXd canditateGenerationH_;
  mutable Eigen::MatrixXd canditateGenerationDifVec_;
  mutable Eigen::MatrixXd canditateGenerationPy_;
  mutable Eigen::EigenSolver<Eigen::MatrixXd> candidateGenerationES_;

  mutable MultilevelPatchAlignment<mtState::nLevels_, mtState::patchSize_> alignment_; /**<Patch aligner*/
  mutable cv::Mat drawImg_;                                                            /**<Image currently used for drawing*/

  const float max_8bit_image_val = 255.0;   /**<CUSTOMIZATION: Max value for 8-bit image in float images. Used for disambiguation between conversion from float to 8-bit or 16-bit images.*/
  std::vector<cv::Mat> projectedDepthImgs_; /**<CUSTOMIZATION: Vector containing LiDAR images projected to each camera frame - Current implementation support 1-Camera*/
  const int min_non_zero_neighbors_ = 4;    /**<CUSTOMIZATION: Minimum number of non-zero neighbors in a depth patch to be considered valid for a depth calculation*/

  /** \brief Constructor.
   *
   *   Loads and sets the needed parameters.
   */
  ImgUpdate()
      : transformFeatureOutputCT_(nullptr),
        pixelOutputCov_((int)(PixelOutput::D_), (int)(PixelOutput::D_)),
        featureOutputCov_((int)(FeatureOutput::D_), (int)(FeatureOutput::D_)),
        featureOutputJac_((int)(FeatureOutput::D_), (int)(mtState::D_)),
        canditateGenerationH_(2, (int)(mtState::D_)),
        canditateGenerationDifVec_((int)(mtState::D_), 1),
        canditateGenerationPy_(2, 2) {
    mpMultiCamera_ = nullptr;
    initCovFeature_.setIdentity();
    initDepth_ = 0.5;
    startLevel_ = 3;
    endLevel_ = 1;
    startDetectionTh_ = 0.9;
    nDetectionBuckets_ = 100;
    fastDetectionThreshold_ = 10;
    scoreDetectionExponent_ = 0.5;
    penaltyDistance_ = 20;
    zeroDistancePenalty_ = nDetectionBuckets_ * 1.0;
    useDirectMethod_ = true;
    doFrameVisualisation_ = true;
    visualizePatches_ = false;
    verbose_ = false;
    trackingUpperBound_ = 0.9;
    trackingLowerBound_ = 0.1;
    minTrackedAndFreeFeatures_ = 0.5;
    minRelativeSTScore_ = 0.2;
    minAbsoluteSTScore_ = 0.2;
    minTimeBetweenPatchUpdate_ = 1.0;
    patchRejectionTh_ = 10.0;
    removeNegativeFeatureAfterUpdate_ = true;
    doVisualMotionDetection_ = false;
    rateOfMovingFeaturesTh_ = 0.5;
    pixelCoordinateMotionTh_ = 1.0;
    minFeatureCountForNoMotionDetection_ = 5;
    minTimeForZeroVelocityUpdate_ = 1.0;
    maxUncertaintyToDepthRatioForDepthInitialization_ = 0.3;
    updateNoisePix_ = 2;
    updateNoiseInt_ = 10000;
    noiseGainForOffCamera_ = 4.0;
    alignConvergencePixelRange_ = 1.0;
    alignCoverageRatio_ = 2.0;
    alignMaxUniSample_ = 5;
    useCrossCameraMeasurements_ = true;
    doStereoInitialization_ = true;
    removalFactor_ = 1.1;
    alignmentGaussianWeightingSigma_ = 2.0;
    discriminativeSamplingDistance_ = 0.0;
    discriminativeSamplingGain_ = 0.0;
    maxAllowedFeatureDistance_ = 25.0;    //CUTOMIZATION
    minTimeBeforeFeaDistCheck_ = 1.0;     //CUTOMIZATION
    useDepthFromLiDAR_ = false;           //CUTOMIZATION
    lidarDepthCalcMode_ = 0;              //CUTOMIZATION
    depthSearchWindowWidth_ = 23;         //CUSTOMIZATION
    depthSearchWindowHeight_ = 47;        //CUSTOMIZATION
    directDepInitCovScaleFactor_ = 1e-3;  //CUTOMIZATION

    doubleRegister_.registerDiagonalMatrix("initCovFeature", initCovFeature_);
    doubleRegister_.registerScalar("initDepth", initDepth_);
    doubleRegister_.registerScalar("startDetectionTh", startDetectionTh_);
    doubleRegister_.registerScalar("scoreDetectionExponent", scoreDetectionExponent_);
    doubleRegister_.registerScalar("penaltyDistance", penaltyDistance_);
    doubleRegister_.registerScalar("zeroDistancePenalty", zeroDistancePenalty_);
    doubleRegister_.registerScalar("trackingUpperBound", trackingUpperBound_);
    doubleRegister_.registerScalar("trackingLowerBound", trackingLowerBound_);
    doubleRegister_.registerScalar("minTrackedAndFreeFeatures", minTrackedAndFreeFeatures_);
    doubleRegister_.registerScalar("minRelativeSTScore", minRelativeSTScore_);
    doubleRegister_.registerScalar("minAbsoluteSTScore", minAbsoluteSTScore_);
    doubleRegister_.registerScalar("minTimeBetweenPatchUpdate", minTimeBetweenPatchUpdate_);
    doubleRegister_.registerScalar("patchRejectionTh", patchRejectionTh_);
    doubleRegister_.registerScalar("MotionDetection.rateOfMovingFeaturesTh", rateOfMovingFeaturesTh_);
    doubleRegister_.registerScalar("MotionDetection.pixelCoordinateMotionTh", pixelCoordinateMotionTh_);
    doubleRegister_.registerScalar("maxUncertaintyToDepthRatioForDepthInitialization", maxUncertaintyToDepthRatioForDepthInitialization_);
    doubleRegister_.registerScalar("alignConvergencePixelRange", alignConvergencePixelRange_);
    doubleRegister_.registerScalar("alignCoverageRatio", alignCoverageRatio_);
    doubleRegister_.registerScalar("removalFactor", removalFactor_);
    doubleRegister_.registerScalar("discriminativeSamplingDistance", discriminativeSamplingDistance_);
    doubleRegister_.registerScalar("discriminativeSamplingGain", discriminativeSamplingGain_);
    intRegister_.registerScalar("fastDetectionThreshold", fastDetectionThreshold_);
    intRegister_.registerScalar("startLevel", startLevel_);
    intRegister_.registerScalar("endLevel", endLevel_);
    intRegister_.registerScalar("nDetectionBuckets", nDetectionBuckets_);
    intRegister_.registerScalar("MotionDetection.minFeatureCountForNoMotionDetection", minFeatureCountForNoMotionDetection_);
    intRegister_.registerScalar("alignMaxUniSample", alignMaxUniSample_);
    boolRegister_.registerScalar("MotionDetection.isEnabled", doVisualMotionDetection_);
    boolRegister_.registerScalar("useDirectMethod", useDirectMethod_);
    boolRegister_.registerScalar("doFrameVisualisation", doFrameVisualisation_);
    boolRegister_.registerScalar("visualizePatches", visualizePatches_);
    boolRegister_.registerScalar("removeNegativeFeatureAfterUpdate", removeNegativeFeatureAfterUpdate_);
    boolRegister_.registerScalar("useCrossCameraMeasurements", useCrossCameraMeasurements_);
    boolRegister_.registerScalar("doStereoInitialization", doStereoInitialization_);
    boolRegister_.registerScalar("useIntensityOffsetForAlignment", alignment_.useIntensityOffset_);
    boolRegister_.registerScalar("useIntensitySqewForAlignment", alignment_.useIntensitySqew_);
    doubleRegister_.removeScalarByVar(updnoiP_(0, 0));
    doubleRegister_.removeScalarByVar(updnoiP_(1, 1));
    doubleRegister_.registerScalar("UpdateNoise.pix", updateNoisePix_);
    doubleRegister_.registerScalar("UpdateNoise.int", updateNoiseInt_);
    doubleRegister_.registerScalar("noiseGainForOffCamera", noiseGainForOffCamera_);
    useImprovedJacobian_ = false;  // TODO: adapt/test
    isZeroVelocityUpdateEnabled_ = false;
    Base::PropertyHandler::registerSubHandler("ZeroVelocityUpdate", zeroVelocityUpdate_);
    zeroVelocityUpdate_.outlierDetection_.registerToPropertyHandler(&zeroVelocityUpdate_, "MahalanobisTh");
    zeroVelocityUpdate_.doubleRegister_.registerScalar("minNoMotionTime", minTimeForZeroVelocityUpdate_);
    zeroVelocityUpdate_.boolRegister_.registerScalar("isEnabled", isZeroVelocityUpdateEnabled_);
    doubleRegister_.removeScalarByStr("alpha");
    doubleRegister_.removeScalarByStr("beta");
    doubleRegister_.removeScalarByStr("kappa");
    alignmentHuberNormThreshold_ = static_cast<double>(alignment_.huberNormThreshold_);
    doubleRegister_.registerScalar("alignmentHuberNormThreshold", alignmentHuberNormThreshold_);
    doubleRegister_.registerScalar("alignmentGaussianWeightingSigma", alignmentGaussianWeightingSigma_);
    alignmentGradientExponent_ = static_cast<double>(alignment_.gradientExponent_);
    doubleRegister_.registerScalar("alignmentGradientExponent", alignmentGradientExponent_);
    doubleRegister_.registerScalar("maxAllowedFeatureDistance", maxAllowedFeatureDistance_);      //CUSTOMIZATION
    doubleRegister_.registerScalar("minTimeBeforeFeaDistCheck", minTimeBeforeFeaDistCheck_);      //CUSTOMIZATION
    boolRegister_.registerScalar("useDepthFromLiDAR", useDepthFromLiDAR_);                        //CUSTOMIZATION
    intRegister_.registerScalar("lidarDepthCalcMode_", lidarDepthCalcMode_);                      //CUSTOMIZATION
    intRegister_.registerScalar("depthSearchWindowWidth", depthSearchWindowWidth_);               //CUSTOMIZATION
    intRegister_.registerScalar("depthSearchWindowHeight", depthSearchWindowHeight_);             //CUSTOMIZATION
    doubleRegister_.registerScalar("directDepInitCovScaleFactor", directDepInitCovScaleFactor_);  //CUSTOMIZATION
  };

  /** \brief Destructor
   */
  virtual ~ImgUpdate(){};

  /** \brief Refresh the properties of the property handler
   */
  void refreshProperties() {
    if (isZeroVelocityUpdateEnabled_) assert(doVisualMotionDetection_);
    if (useDirectMethod_) {
      updnoiP_.setIdentity();
      updnoiP_ = updnoiP_ * updateNoiseInt_;
    } else {
      updnoiP_.setIdentity();
      updnoiP_ = updnoiP_ * updateNoisePix_;
    }
    alignment_.huberNormThreshold_ = static_cast<float>(alignmentHuberNormThreshold_);
    alignment_.computeWeightings(alignmentGaussianWeightingSigma_);
    alignment_.gradientExponent_ = static_cast<float>(alignmentGradientExponent_);
  };

  /** \brief Sets the multicamera pointer
   *
   * @param mpMultiCamera - Multicamera pointer
   */
  void setCamera(MultiCamera<mtState::nCam_>* mpMultiCamera) {
    mpMultiCamera_ = mpMultiCamera;
    transformFeatureOutputCT_.mpMultiCamera_ = mpMultiCamera;
  }

  /** \brief Sets the innovation term.
   *
   *  \note If \ref useDirectMethod_ is set true, the innovation term is based directly on pixel intensity errors.
   *  \note If \ref useDirectMethod_ is set false, the reprojection error is used for the innovation term.
   *  @param mtInnovation - Class, holding innovation data.
   *  @param state        - Filter %State.
   *  @param noise        - Additive discrete Gaussian noise.
   */
  void evalInnovation(mtInnovation& y, const mtState& state, const mtNoise& noise) const {
    const int& ID = state.aux().activeFeature_;
    const int& camID = state.CfP(ID).camID_;
    const int activeCamID = (state.aux().activeCameraCounter_ + camID) % mtState::nCam_;
    transformFeatureOutputCT_.setFeatureID(ID);
    transformFeatureOutputCT_.setOutputCameraID(activeCamID);
    transformFeatureOutputCT_.transformState(state, featureOutput_);

    if (useDirectMethod_) {
      if (doFrameVisualisation_ && featureOutput_.c().com_c()) {
        if (activeCamID == camID) {
          featureOutput_.c().drawPoint(drawImg_, cv::Scalar(0, 175, 175));
        } else {
          featureOutput_.c().drawPoint(drawImg_, cv::Scalar(175, 175, 0));
        }
      }
      if (alignment_.getLinearAlignEquationsReduced(meas_.aux().pyr_[activeCamID], *state.aux().mpCurrentFeature_->mpMultilevelPatch_, featureOutput_.c(), endLevel_, startLevel_, A_red_, b_red_)) {
        y.template get<mtInnovation::_pix>() = b_red_ + noise.template get<mtNoise::_pix>();
        if (verbose_) {
          std::cout << "    \033[32mMaking update with feature " << ID << " from camera " << camID << " in camera " << activeCamID << "\033[0m" << std::endl;
        }
      } else {
        y.template get<mtInnovation::_pix>() = noise.template get<mtNoise::_pix>();
        if (verbose_) {
          std::cout << "    \033[31mFailed Construction of Alignment Equations with feature " << ID << " from camera " << camID << " in camera " << activeCamID << "\033[0m" << std::endl;
        }
        cancelIteration_ = true;
      }
    } else {
      Eigen::Vector2d pixError;
      pixError(0) = static_cast<double>(state.aux().feaCoorMeas_[ID].get_c().x - featureOutput_.c().get_c().x);
      pixError(1) = static_cast<double>(state.aux().feaCoorMeas_[ID].get_c().y - featureOutput_.c().get_c().y);
      y.template get<mtInnovation::_pix>() = pixError + noise.template get<mtNoise::_pix>();
    }
  }

  bool generateCandidates(const mtFilterState& filterState, mtState& candidate) const {
    candidate = filterState.state_;

    if (candidateCounter_ == 0) {
      const int& ID = candidate.aux().activeFeature_;
      const int& camID = candidate.CfP(ID).camID_;
      const int activeCamID = (candidate.aux().activeCameraCounter_ + camID) % mtState::nCam_;
      transformFeatureOutputCT_.setFeatureID(ID);
      transformFeatureOutputCT_.setOutputCameraID(activeCamID);
      transformFeatureOutputCT_.transformState(candidate, featureOutput_);
      transformFeatureOutputCT_.jacTransform(featureOutputJac_, candidate);
      mpMultiCamera_->cameras_[activeCamID].bearingToPixel(featureOutput_.c().get_nor(), c_temp_, c_J_);

      canditateGenerationH_ = -c_J_ * featureOutputJac_.template block<2, mtState::D_>(0, 0);
      canditateGenerationPy_ = canditateGenerationH_ * filterState.cov_ * canditateGenerationH_.transpose();
      candidateGenerationES_.compute(canditateGenerationPy_);
    }

    while (++candidateCounter_) {
      int u = (candidateCounter_ - 1) / (2 * alignMaxUniSample_ + 1) - alignMaxUniSample_;
      if (u > alignMaxUniSample_)
        break;
      int v = (candidateCounter_ - 1) % (2 * alignMaxUniSample_ + 1) - alignMaxUniSample_;
      if (pow(u * alignConvergencePixelRange_, 2) / candidateGenerationES_.eigenvalues()(0).real() + pow(v * alignConvergencePixelRange_, 2) / candidateGenerationES_.eigenvalues()(1).real() < pow(alignCoverageRatio_, 2)) {
        Eigen::Vector2d dy = u * alignConvergencePixelRange_ * candidateGenerationES_.eigenvectors().col(0).real() + v * alignConvergencePixelRange_ * candidateGenerationES_.eigenvectors().col(1).real();
        canditateGenerationDifVec_ = -filterState.cov_ * canditateGenerationH_.transpose() * canditateGenerationPy_.inverse() * dy;
        candidate.boxPlus(canditateGenerationDifVec_, candidate);
        return true;
      }
    }
    return false;
  }

  bool extraOutlierCheck(const mtState& state) const {
    const int& ID = state.aux().activeFeature_;
    const int& camID = state.CfP(ID).camID_;
    const int activeCamID = (state.aux().activeCameraCounter_ + camID) % mtState::nCam_;
    transformFeatureOutputCT_.setFeatureID(ID);
    transformFeatureOutputCT_.setOutputCameraID(activeCamID);
    transformFeatureOutputCT_.transformState(state, featureOutput_);

    if (!hasConverged_) {
      if (verbose_) std::cout << "    \033[31mREJECTED (iterations did no converge)\033[0m" << std::endl;
      if (mlpTemp1_.isMultilevelPatchInFrame(meas_.aux().pyr_[activeCamID], featureOutput_.c(), startLevel_, false)) {
        featureOutput_.c().drawPoint(drawImg_, cv::Scalar(255, 0, 0), 1.0);
      }
      return false;
    }

    if (patchRejectionTh_ >= 0) {
      if (!mlpTemp1_.isMultilevelPatchInFrame(meas_.aux().pyr_[activeCamID], featureOutput_.c(), startLevel_, false)) {
        if (verbose_) std::cout << "    \033[31mREJECTED (not in frame)\033[0m" << std::endl;
        return false;
      }
      mlpTemp1_.extractMultilevelPatchFromImage(meas_.aux().pyr_[activeCamID], featureOutput_.c(), startLevel_, false);
      const float avgError = mlpTemp1_.computeAverageDifference(*state.aux().mpCurrentFeature_->mpMultilevelPatch_, endLevel_, startLevel_);
      if (avgError > patchRejectionTh_) {
        if (verbose_) std::cout << "    \033[31mREJECTED (error too large: " << avgError << ")\033[0m" << std::endl;
        featureOutput_.c().drawPoint(drawImg_, cv::Scalar(255, 255, 0), 1.0);
        return false;
      }

      // Use 4 sample around feature, at least two should be above the treshold
      if (discriminativeSamplingDistance_ > 0.0) {
        FeatureOutput sample;
        V3D d;
        int countAboveThreshold = 0;
        for (int i = 0; i < 4; i++) {
          d.setZero();
          d(i % 2) = (i / 2 * 2 - 1) * discriminativeSamplingDistance_;
          featureOutput_.boxPlus(d, sample);
          if (mlpTemp1_.isMultilevelPatchInFrame(meas_.aux().pyr_[activeCamID], sample.c(), startLevel_, false)) {
            mlpTemp1_.extractMultilevelPatchFromImage(meas_.aux().pyr_[activeCamID], sample.c(), startLevel_, false);
            const float sampleError = mlpTemp1_.computeAverageDifference(*state.aux().mpCurrentFeature_->mpMultilevelPatch_, endLevel_, startLevel_);
            const bool isAboveThreshold = (discriminativeSamplingGain_ <= 1.0 & sampleError > patchRejectionTh_) | (discriminativeSamplingGain_ > 1.0 & sampleError > discriminativeSamplingGain_ * avgError);
            countAboveThreshold += isAboveThreshold;
            if (isAboveThreshold) {
              sample.c().drawPoint(drawImg_, cv::Scalar(0, 255, 0), 2.0);
            } else {
              sample.c().drawPoint(drawImg_, cv::Scalar(0, 0, 255), 2.0);
            }
          }
        }
        if (countAboveThreshold < 2) {
          if (verbose_) std::cout << "    \033[31mREJECTED (feature location not discriminative enough)\033[0m" << std::endl;
          return false;
        }
      }
    }

    featureOutput_.c().drawPoint(drawImg_, cv::Scalar(0, 0, 255), 1.0);
    return true;
  }

  /** \brief Computes the Jacobian for the update step of the filter.
   *
   *  \note If \ref useDirectMethod_ is set true, the jacobian is set w.r.t. the intensity errors.
   *  \note If \ref useDirectMethod_ is set false, the jacobian is set w.r.t. the reprojection error.
   *  @param F     - Jacobian for the update step of the filter.
   *  @param state - Filter state.
   */
  void jacState(MXD& F, const mtState& state) const {
    const int& ID = state.aux().activeFeature_;
    const int& camID = state.CfP(ID).camID_;
    const int activeCamID = (state.aux().activeCameraCounter_ + camID) % mtState::nCam_;
    transformFeatureOutputCT_.setFeatureID(ID);
    transformFeatureOutputCT_.setOutputCameraID(activeCamID);
    transformFeatureOutputCT_.transformState(state, featureOutput_);

    if (useDirectMethod_) {
      if (alignment_.getLinearAlignEquationsReduced(meas_.aux().pyr_[activeCamID], *state.aux().mpCurrentFeature_->mpMultilevelPatch_, featureOutput_.c(), endLevel_, startLevel_, A_red_, b_red_)) {
        transformFeatureOutputCT_.jacTransform(featureOutputJac_, state);
        mpMultiCamera_->cameras_[activeCamID].bearingToPixel(featureOutput_.c().get_nor(), c_temp_, c_J_);
        F = -A_red_ * c_J_ * featureOutputJac_.template block<2, mtState::D_>(0, 0);
      } else {
        F.setZero();
        cancelIteration_ = true;
      }
    } else {
      transformFeatureOutputCT_.jacTransform(featureOutputJac_, state);
      mpMultiCamera_->cameras_[activeCamID].bearingToPixel(featureOutput_.c().get_nor(), c_temp_, c_J_);
      F = -c_J_ * featureOutputJac_.template block<2, mtState::D_>(0, 0);
    }
  }

  /** \brief Computes the Jacobian for the update step of the filter.
   *
   *  @param G     - Jacobian for the update step of the filter.
   *  @param state - Filter state.
   */
  void jacNoise(MXD& G, const mtState& state) const {
    G.setZero();
    G.template block<2, 2>(mtInnovation::template getId<mtInnovation::_pix>(), mtNoise::template getId<mtNoise::_pix>()) = Eigen::Matrix2d::Identity();
  }

  /** \brief Prepares the filter state for the update.
   *
   *   @param filterState - Filter state.
   *   @param meas        - Update measurement.
   *   @todo sort feature by covariance and use more accurate ones first
   */
  void commonPreProcess(mtFilterState& filterState, const mtMeas& meas) {
    assert(filterState.t_ == meas.aux().imgTime_);
    for (int i = 0; i < mtState::nCam_; i++) {
      if (doFrameVisualisation_) {
        //CUSTOMIZATION
        //Check if image needs to be normalized for display purposes
        double imgMin, imgMax;
        cv::minMaxLoc(meas.aux().pyr_[i].imgs_[startLevel_], &imgMin, &imgMax);  //Check on the highest allowed pyramid level to save time, Equaliztion applied on original image
        cv::Mat tmpImg;
        if (imgMax > max_8bit_image_val)
          meas.aux().pyr_[i].equalizeHistogram(0, tmpImg);
        else
          meas.aux().pyr_[i].imgs_[0].convertTo(tmpImg, CV_8UC1);
        //Convert Image to color for drawing purposes
        cvtColor(tmpImg, filterState.img_[i], cv::COLOR_GRAY2RGB);
        //CUSTOMIZATION
      }
    }
    filterState.imgTime_ = filterState.t_;
    filterState.imageCounter_++;
    if (visualizePatches_) {
      filterState.patchDrawing_ = cv::Mat::zeros(mtState::nMax_ * filterState.drawPS_, (1 + 2 * mtState::nCam_) * filterState.drawPS_, CV_8UC3);
    }
    filterState.state_.aux().activeFeature_ = 0;
    filterState.state_.aux().activeCameraCounter_ = 0;

    // CUSTOMIZATION
    if (useDepthFromLiDAR_) {
      // Project pointcloud from lidar frame to image frame
      projectedDepthImgs_.clear();
      projectedDepthImgs_.resize(mtState::nCam_);
      for (int camID = 0; camID < mtState::nCam_; ++camID) {
        projectedDepthImgs_[camID] = cv::Mat::zeros(meas.aux().pyr_[camID].imgs_[0].rows, meas.aux().pyr_[camID].imgs_[0].cols, CV_32FC1);
        std::clock_t projectPCLtoCameraFrame_t = std::clock();
        projectPCLtoImgFrame(meas.aux().pclCloud_, meas.aux().pcl2imgTF_, camID, projectedDepthImgs_[camID]);
        if (verbose_) std::cout << " Timing - projectPCLtoImgFrame: " << double(std::clock() - projectPCLtoCameraFrame_t) / CLOCKS_PER_SEC * 1000.0
                                << " ms - for CamID:" << camID
                                << " NonZero DepthImg pts: " << cv::countNonZero(projectedDepthImgs_[camID]) << std::endl;
      }
    }

    //DEBUG
    /*for (int camID = 0; camID < mtState::nCam_; camID++) {
      for (unsigned int ID = 0; ID < mtState::nMax_; ID++) {
        // Update status and visualization
        if (filterState.fsm_.isValid_[ID]) {
          transformFeatureOutputCT_.setFeatureID(ID);
          transformFeatureOutputCT_.setOutputCameraID(camID);
          transformFeatureOutputCT_.transformState(filterState.state_, featureOutput_);
          transformFeatureOutputCT_.transformCovMat(filterState.state_, filterState.cov_, featureOutputCov_);
          std::cout << "FeaID: " << ID
                    << ", px(x,y):" << cvRound(featureOutput_.c().get_c().x) << "," << cvRound(featureOutput_.c().get_c().y)
                    << ", dist: " << featureOutput_.d().getDistance()
                    << ", Direct:" << (filterState.fsm_.features_[ID].mpCoordinates_->directDepthInit_ ? "True" : "False")
                    << ", CovMat\n: " << featureOutputCov_ << std::endl;
        }
      }
    }*/
    //DEBUG
    // CUSTOMIZATION

    /* Detect Image changes by looking at the feature patches between current and previous image (both at the current feature location)
     * The maximum change of intensity is obtained if the pixel is moved along the strongest gradient.
     * The maximal singularvalue, which is equivalent to the root of the larger eigenvalue of the Hessian,
     * gives us range in which intensity change is allowed to be.
     */
    if (doVisualMotionDetection_ && filterState.imageCounter_ > 1) {
      int totCountInFrame = 0;
      int totCountInMotion = 0;
      for (unsigned int i = 0; i < mtState::nMax_; i++) {
        if (filterState.fsm_.isValid_[i]) {
          const int& camID = filterState.state_.CfP(i).camID_;  // Camera ID of the feature.
          tempCoordinates_ = *filterState.fsm_.features_[i].mpCoordinates_;
          tempCoordinates_.set_warp_identity();
          if (mlpTemp1_.isMultilevelPatchInFrame(filterState.prevPyr_[camID], tempCoordinates_, startLevel_, true)) {
            mlpTemp1_.extractMultilevelPatchFromImage(filterState.prevPyr_[camID], tempCoordinates_, startLevel_, true);
            mlpTemp1_.computeMultilevelShiTomasiScore(endLevel_, startLevel_);
            mlpTemp2_.extractMultilevelPatchFromImage(meas.aux().pyr_[camID], tempCoordinates_, startLevel_, true);
            const float avgError = mlpTemp1_.computeAverageDifference(mlpTemp2_, endLevel_, startLevel_);
            if (avgError / std::sqrt(mlpTemp1_.e1_) > static_cast<float>(pixelCoordinateMotionTh_)) totCountInMotion++;
            totCountInFrame++;
          }
        }
      }
      if (rateOfMovingFeaturesTh_ / totCountInMotion * totCountInFrame < 1.0 || totCountInFrame < minFeatureCountForNoMotionDetection_) {
        filterState.state_.aux().timeSinceLastImageMotion_ = 0.0;
      }
    }
  }

  /** \brief Pre-Processing for the image update.
   *
   *  Summary:
   *  1. Searches a valid MultilevelPatchFeature from the filter state.
   *  2. Transforms the MultilevelPatchFeature into the target frame.
   *  3. Executes a 2D patch alignment in the target frame. If unsuccessful go back to step 1.
   *  4. If bearing error between aligned patch and estimated patch too large, alter linearization point.
   *     Bearing vector in the state is directly altered to the new aligned position.
   *
   *  @param filterState - Filter state.
   *  @param meas        - Update measurement.
   *  @param isFinished  - True, if process has finished.
   *  @todo split into methods
   */
  void preProcess(mtFilterState& filterState, const mtMeas& meas, bool& isFinished) {
    if (isFinished) {  // gets called if this is the first call
      commonPreProcess(filterState, meas);
      isFinished = false;
    }
    bool foundValidMeasurement = false;
    typename mtFilterState::mtState& state = filterState.state_;
    MXD& cov = filterState.cov_;
    int& ID = filterState.state_.aux().activeFeature_;  // ID of the current updated feature!!! Initially set to 0.
    int& activeCamCounter = filterState.state_.aux().activeCameraCounter_;

    // Actualize camera extrinsics (gets also update in calls to TransformFeatureOutputCT)
    state.updateMultiCameraExtrinsics(mpMultiCamera_);

    while (ID < mtState::nMax_ && foundValidMeasurement == false) {
      if (filterState.fsm_.isValid_[ID]) {
        // Data handling stuff
        FeatureManager<mtState::nLevels_, mtState::patchSize_, mtState::nCam_>& f = filterState.fsm_.features_[ID];
        const int camID = f.mpCoordinates_->camID_;
        const int activeCamID = (activeCamCounter + camID) % mtState::nCam_;
        drawImg_ = filterState.img_[activeCamID];
        if (activeCamCounter == 0) {
          f.mpStatistics_->increaseStatistics(filterState.t_);
          if (verbose_) {
            std::cout << "=========== Feature " << f.idx_ << " ==================================================== " << std::endl;
          }
          // Visualize patch tracking
          if (visualizePatches_) {
            f.mpMultilevelPatch_->drawMultilevelPatch(filterState.patchDrawing_, cv::Point2i(2, filterState.drawPB_ + ID * filterState.drawPS_), 1, false);
            cv::putText(filterState.patchDrawing_, std::to_string(f.idx_), cv::Point2i(2, 10 + ID * filterState.drawPS_), cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(255, 255, 255));
          }
          f.log_prediction_ = *f.mpCoordinates_;
        }
        if (verbose_) {
          std::cout << "  ========== Camera  " << activeCamID << " ================= " << std::endl;
          std::cout << "  Normal in feature frame: " << f.mpCoordinates_->get_nor().getVec().transpose() << std::endl;
          std::cout << "  with depth: " << f.mpDistance_->getDistance() << std::endl;
        }

        // Get coordinates in target frame
        transformFeatureOutputCT_.setFeatureID(ID);
        transformFeatureOutputCT_.setOutputCameraID(activeCamID);
        transformFeatureOutputCT_.transformState(state, featureOutput_);
        transformFeatureOutputCT_.transformCovMat(state, cov, featureOutputCov_);
        if (verbose_) std::cout << "    Normal in camera frame: " << featureOutput_.c().get_nor().getVec().transpose() << std::endl;

        // Check if feature in target frame
        if (!mlpTemp1_.isMultilevelPatchInFrame(filterState.prevPyr_[camID], featureOutput_.c(), startLevel_, false)) {
          f.mpStatistics_->status_[activeCamID] = NOT_IN_FRAME;
          if (verbose_) std::cout << "    NOT in frame" << std::endl;
        } else {
          pixelOutputCT_.transformState(featureOutput_, pixelOutput_);
          pixelOutputCT_.transformCovMat(featureOutput_, featureOutputCov_, pixelOutputCov_);
          featureOutput_.c().setPixelCov(pixelOutputCov_);

          // Visualization
          if (doFrameVisualisation_) {
            if (activeCamID == camID) {
              featureOutput_.c().drawEllipse(drawImg_, cv::Scalar(0, 175, 175), 2.0, true);
              featureOutput_.c().drawText(drawImg_, std::to_string(f.idx_), cv::Scalar(0, 175, 175));
            } else {
              featureOutput_.c().drawEllipse(drawImg_, cv::Scalar(175, 175, 0), 2.0, true);
              featureOutput_.c().drawText(drawImg_, std::to_string(f.idx_), cv::Scalar(175, 175, 0));
            }
          }
          if (visualizePatches_) {
            if (mlpTemp1_.isMultilevelPatchInFrame(meas.aux().pyr_[activeCamID], featureOutput_.c(), startLevel_, false)) {
              mlpTemp1_.extractMultilevelPatchFromImage(meas.aux().pyr_[activeCamID], featureOutput_.c(), startLevel_, false);
              mlpTemp1_.drawMultilevelPatch(filterState.patchDrawing_, cv::Point2i(filterState.drawPB_ + (1 + 2 * activeCamID) * filterState.drawPS_, filterState.drawPB_ + ID * filterState.drawPS_), 1, false);
            }
          }

          if (useDirectMethod_) {
            if (verbose_) std::cout << "    Do direct update (without alignment)" << std::endl;
            state.aux().mpCurrentFeature_ = &filterState.fsm_.features_[ID];
            if (activeCamCounter == 0) {
              updnoiP_.setIdentity();
              updnoiP_ = updnoiP_ * updateNoiseInt_;
            } else {
              updnoiP_.setIdentity();
              updnoiP_ = updnoiP_ * updateNoiseInt_ * noiseGainForOffCamera_;
            }
            foundValidMeasurement = true;
          } else {
            if (alignment_.align2DAdaptive(alignedCoordinates_, meas.aux().pyr_[activeCamID], *f.mpMultilevelPatch_, featureOutput_.c(), startLevel_, endLevel_,
                                           alignConvergencePixelRange_, alignCoverageRatio_, alignMaxUniSample_)) {
              if (verbose_) std::cout << "    Found match: " << alignedCoordinates_.get_nor().getVec().transpose() << std::endl;
              if (mlpTemp1_.isMultilevelPatchInFrame(meas.aux().pyr_[activeCamID], alignedCoordinates_, startLevel_, false)) {
                float avgError = 0.0;
                if (patchRejectionTh_ >= 0) {
                  mlpTemp1_.extractMultilevelPatchFromImage(meas.aux().pyr_[activeCamID], alignedCoordinates_, startLevel_, false);
                  avgError = mlpTemp1_.computeAverageDifference(*f.mpMultilevelPatch_, endLevel_, startLevel_);
                }
                if (patchRejectionTh_ >= 0 && avgError > patchRejectionTh_) {
                  f.mpStatistics_->status_[activeCamID] = FAILED_ALIGNEMENT;
                  if (verbose_) std::cout << "    \033[31mREJECTED (error too large)\033[0m" << std::endl;
                } else {
                  if (doFrameVisualisation_) alignedCoordinates_.drawPoint(drawImg_, cv::Scalar(255, 0, 255));
                  state.aux().feaCoorMeas_[ID] = alignedCoordinates_;
                  foundValidMeasurement = true;
                }
              } else {
                f.mpStatistics_->status_[activeCamID] = FAILED_ALIGNEMENT;
                if (verbose_) std::cout << "    \033[31mNot in frame after alignment\033[0m" << std::endl;
              }
            } else {
              f.mpStatistics_->status_[activeCamID] = FAILED_ALIGNEMENT;
              if (verbose_) std::cout << "    \033[31mNOT FOUND (matching failed)\033[0m" << std::endl;
            }
          }
        }
      }
      if (foundValidMeasurement == false) {
        activeCamCounter++;
        if (activeCamCounter == mtState::nCam_ || !useCrossCameraMeasurements_) {
          activeCamCounter = 0;
          ID++;
        }
      }
    }  // while end
    if (ID >= mtState::nMax_) {
      isFinished = true;
    }
  };

  /** \brief Post-Processing for the image update.
   *
   *  Summary:
   *  1. Some drawing tasks.
   *  2. Removal of features with negative depth.
   *
   *  @param filterState      - Filter state.
   *  @param meas             - Update measurement.
   *  @param outlierDetection - Outlier detection.
   *  @param isFinished       - True, if process has finished.
   */
  void postProcess(mtFilterState& filterState, const mtMeas& meas, const mtOutlierDetection& outlierDetection, bool& isFinished) {
    int& ID = filterState.state_.aux().activeFeature_;  // Get the ID of the updated feature.
    int& activeCamCounter = filterState.state_.aux().activeCameraCounter_;

    if (isFinished) {
      commonPostProcess(filterState, meas);
    } else {
      FeatureManager<mtState::nLevels_, mtState::patchSize_, mtState::nCam_>& f = filterState.fsm_.features_[ID];
      const int camID = f.mpCoordinates_->camID_;
      const int activeCamID = (activeCamCounter + camID) % mtState::nCam_;

      // Remove negative feature
      if (removeNegativeFeatureAfterUpdate_) {
        for (unsigned int i = 0; i < mtState::nMax_; i++) {
          if (filterState.fsm_.isValid_[i]) {
            if (filterState.state_.dep(i).getDistance() < 1e-8 ||
                ((filterState.fsm_.features_[i].mpStatistics_->currentTime_ - filterState.fsm_.features_[i].mpStatistics_->initTime_) > minTimeBeforeFeaDistCheck_ &&
                 filterState.state_.dep(i).getDistance() > maxAllowedFeatureDistance_))  //CUSTOMIZATION delete feature if it has more than minTimeBeforeMaxFeaDistCheck secs of updates and converged depth is more than max allowed.)
            {
              if (verbose_) std::cout << "    \033[33mRemoved feature " << filterState.fsm_.features_[i].idx_ << " with invalid distance parameter " << filterState.state_.dep(i).p_ << "!\033[0m" << std::endl;
              filterState.fsm_.isValid_[i] = false;
              filterState.resetFeatureCovariance(i, Eigen::Matrix3d::Identity());
            }
          }
        }
      }

      if (filterState.fsm_.isValid_[ID]) {
        // Update status and visualization
        transformFeatureOutputCT_.setFeatureID(ID);
        transformFeatureOutputCT_.setOutputCameraID(activeCamID);
        transformFeatureOutputCT_.transformState(filterState.state_, featureOutput_);

        // Draw information ellipse
        bool doInformationGainVizualization = false;
        if (doFrameVisualisation_ && doInformationGainVizualization) {
          MXD F(2, 2);
          F = A_red_;
          F = F.transpose() * F * 1.0 / updateNoiseInt_;
          featureOutput_.c().setPixelCov(F);
          featureOutput_.c().drawEllipse(drawImg_, cv::Scalar(0, 255, 0), 10, false);
          F.setIdentity();
          F = F.transpose() * F * 1.0 / updateNoisePix_;
          featureOutput_.c().setPixelCov(F);
          featureOutput_.c().drawEllipse(drawImg_, cv::Scalar(0, 0, 255), 10, false);
        }
        filterState.mlpErrorLog_[ID] = alignment_.mlpError_;

        if ((filterState.mode_ == LWF::ModeIEKF && successfulUpdate_) || (filterState.mode_ == LWF::ModeEKF && !outlierDetection.isOutlier(0))) {
          if (mlpTemp1_.isMultilevelPatchInFrame(meas.aux().pyr_[camID], featureOutput_.c(), startLevel_, false)) {
            f.mpStatistics_->status_[activeCamID] = TRACKED;
            if (doFrameVisualisation_) {
              //CUSTOMIZATION - Drawing Depth Initialized Patch(Purple) with Depth Window(Cyan)
              if (filterState.fsm_.features_[ID].mpCoordinates_->directDepthInit_) {
                mlpTemp1_.drawMultilevelPatchBorder(drawImg_, featureOutput_.c(), 1.0, cv::Scalar(255, 0, 255));
                cv::rectangle(drawImg_,
                              cv::Point2f(featureOutput_.c().get_c().x - (depthSearchWindowWidth_ / 2), featureOutput_.c().get_c().y - (depthSearchWindowHeight_ / 2)),
                              cv::Point2f(featureOutput_.c().get_c().x + (depthSearchWindowWidth_ / 2), featureOutput_.c().get_c().y + (depthSearchWindowHeight_ / 2)),
                              cv::Scalar(255, 255, 0));
              } else
                mlpTemp1_.drawMultilevelPatchBorder(drawImg_, featureOutput_.c(), 1.0, cv::Scalar(0, 150 + (activeCamID == camID) * 105, 0));
              //CUSTOMIZATION
            }
          } else {
            f.mpStatistics_->status_[activeCamID] = FAILED_TRACKING;
            if (doFrameVisualisation_) {
              mlpTemp1_.drawMultilevelPatchBorder(drawImg_, featureOutput_.c(), 1.0, cv::Scalar(0, 0, 150 + (activeCamID == camID) * 105));
              featureOutput_.c().drawText(drawImg_, "NIF", cv::Scalar(0, 0, 150 + (activeCamID == camID) * 105));
            }
            if (verbose_) std::cout << "    \033[31mNot in frame after update!\033[0m" << std::endl;
          }
        } else {
          f.mpStatistics_->status_[activeCamID] = FAILED_TRACKING;
          if (outlierDetection.isOutlier(0)) {
            if (doFrameVisualisation_) {
              mlpTemp1_.drawMultilevelPatchBorder(drawImg_, featureOutput_.c(), 1.0, cv::Scalar(0, 0, 150 + (activeCamID == camID) * 105));
              featureOutput_.c().drawText(drawImg_, "MD: " + std::to_string(outlierDetection.getMahalDistance(0)), cv::Scalar(0, 0, 150 + (activeCamID == camID) * 105));
            }
            if (verbose_) std::cout << "    \033[31mRecognized as outlier by filter: " << outlierDetection.getMahalDistance(0) << "\033[0m" << std::endl;
          } else if (!hasConverged_) {
            if (doFrameVisualisation_) {
              mlpTemp1_.drawMultilevelPatchBorder(drawImg_, featureOutput_.c(), 1.0, cv::Scalar(0, 0, 150 + (activeCamID == camID) * 105));
              featureOutput_.c().drawText(drawImg_, "INC", cv::Scalar(0, 0, 150 + (activeCamID == camID) * 105));
            }
            if (verbose_) std::cout << "    \033[31mIterations not converged!\033[0m" << std::endl;
          } else {
            if (doFrameVisualisation_) {
              mlpTemp1_.drawMultilevelPatchBorder(drawImg_, featureOutput_.c(), 1.0, cv::Scalar(0, 0, 150 + (activeCamID == camID) * 105));
              featureOutput_.c().drawText(drawImg_, "PE", cv::Scalar(0, 0, 150 + (activeCamID == camID) * 105));
            }
            if (verbose_) std::cout << "    \033[31mToo large pixel intesity error!\033[0m" << std::endl;
          }
        }

        // Visualize patch tracking
        if (visualizePatches_) {
          if (mlpTemp1_.isMultilevelPatchInFrame(meas.aux().pyr_[activeCamID], featureOutput_.c(), mtState::nLevels_ - 1, false)) {
            mlpTemp1_.extractMultilevelPatchFromImage(meas.aux().pyr_[activeCamID], featureOutput_.c(), mtState::nLevels_ - 1, false);
            mlpTemp1_.drawMultilevelPatch(filterState.patchDrawing_, cv::Point2i(filterState.drawPB_ + (2 + 2 * activeCamID) * filterState.drawPS_, filterState.drawPB_ + ID * filterState.drawPS_), 1, false);
          }
          if (f.mpStatistics_->status_[activeCamID] == TRACKED) {
            cv::rectangle(filterState.patchDrawing_, cv::Point2i((2 + 2 * activeCamID) * filterState.drawPS_, ID * filterState.drawPS_), cv::Point2i((3 + 2 * activeCamID) * filterState.drawPS_ - 1, (ID + 1) * filterState.drawPS_ - 1), cv::Scalar(0, 255, 0), 1, 8, 0);
          } else {
            cv::rectangle(filterState.patchDrawing_, cv::Point2i((2 + 2 * activeCamID) * filterState.drawPS_, ID * filterState.drawPS_), cv::Point2i((3 + 2 * activeCamID) * filterState.drawPS_ - 1, (ID + 1) * filterState.drawPS_ - 1), cv::Scalar(0, 0, 255), 1, 8, 0);
          }
        }
      }
      activeCamCounter++;
      if (activeCamCounter == mtState::nCam_ || !useCrossCameraMeasurements_) {
        activeCamCounter = 0;
        ID++;
      }
    }
  };

  /** \brief Final Post-Processing step for the image update.
   *
   *  Summary:
   *  1. For each feature in the state: Extract patches and compute Shi-Tomasi Score.
   *  2. Removal of bad features from the state.
   *  3. Get new features and add them to the state.
   *
   *  @param filterState      - Filter state.
   *  @param meas             - Update measurement.
   */
  void commonPostProcess(mtFilterState& filterState, const mtMeas& meas) {
    typename mtFilterState::mtState& state = filterState.state_;
    MXD& cov = filterState.cov_;

    // Actualize camera extrinsics
    state.updateMultiCameraExtrinsics(mpMultiCamera_);

    int countTracked = 0;
    // For all features in the state.
    for (unsigned int i = 0; i < mtState::nMax_; i++) {
      if (filterState.fsm_.isValid_[i]) {
        FeatureManager<mtState::nLevels_, mtState::patchSize_, mtState::nCam_>& f = filterState.fsm_.features_[i];
        const int camID = f.mpCoordinates_->camID_;
        if (f.mpStatistics_->trackedInSomeFrame()) {
          countTracked++;
        }
        if (f.mpStatistics_->status_[camID] == TRACKED && filterState.t_ - f.mpStatistics_->lastPatchUpdate_ > minTimeBetweenPatchUpdate_) {
          tempCoordinates_ = *f.mpCoordinates_;
          tempCoordinates_.set_warp_identity();
          if (mlpTemp1_.isMultilevelPatchInFrame(meas.aux().pyr_[camID], tempCoordinates_, startLevel_, true)) {
            mlpTemp1_.extractMultilevelPatchFromImage(meas.aux().pyr_[camID], tempCoordinates_, startLevel_, true);
            mlpTemp1_.computeMultilevelShiTomasiScore(endLevel_, startLevel_);
            if (mlpTemp1_.s_ >= static_cast<float>(minAbsoluteSTScore_) && mlpTemp1_.s_ >= static_cast<float>(minRelativeSTScore_) * (f.mpMultilevelPatch_->s_)) {
              *f.mpMultilevelPatch_ = mlpTemp1_;
              f.mpCoordinates_->set_warp_identity();
              f.mpStatistics_->lastPatchUpdate_ = filterState.t_;
            }
          }
        }
        // Visualize Quatlity
        if (visualizePatches_) {
          for (int j = 0; j < mtState::nCam_; j++) {
            // Local Quality
            const double qLQ = f.mpStatistics_->getLocalQuality(j);
            cv::line(filterState.patchDrawing_, cv::Point2i((2 + 2 * j) * filterState.drawPS_ + 1, (i + 1) * filterState.drawPS_ - 4), cv::Point2i((2 + 2 * j) * filterState.drawPS_ + 1 + (filterState.drawPS_ - 3) * qLQ, (i + 1) * filterState.drawPS_ - 4), cv::Scalar(0, 255 * qLQ, 255 * (1 - qLQ)), 2, 8, 0);
          }
          const double qALQ = f.mpStatistics_->getAverageLocalQuality();
          cv::line(filterState.patchDrawing_, cv::Point2i(1, (i + 1) * filterState.drawPS_ - 10), cv::Point2i(1 + (filterState.drawPS_ - 3) * qALQ, (i + 1) * filterState.drawPS_ - 10), cv::Scalar(0, 255 * qALQ, 255 * (1 - qALQ)), 2, 8, 0);
          const double qLV = f.mpStatistics_->getJointLocalVisibility();
          cv::line(filterState.patchDrawing_, cv::Point2i(1, (i + 1) * filterState.drawPS_ - 7), cv::Point2i(1 + (filterState.drawPS_ - 3) * qLV, (i + 1) * filterState.drawPS_ - 7), cv::Scalar(0, 255 * qLV, 255 * (1 - qLV)), 2, 8, 0);
          const double qGQ = f.mpStatistics_->getGlobalQuality();
          cv::line(filterState.patchDrawing_, cv::Point2i(1, (i + 1) * filterState.drawPS_ - 4), cv::Point2i(1 + (filterState.drawPS_ - 3) * qGQ, (i + 1) * filterState.drawPS_ - 4), cv::Scalar(0, 255 * qGQ, 255 * (1 - qALQ)), 2, 8, 0);
        }
      }
    }

    // Remove bad feature.
    float averageScore = filterState.fsm_.getAverageScore();  // TODO: make the following dependent on the ST-score
    if (verbose_) std::cout << "Removing features: ";
    for (unsigned int i = 0; i < mtState::nMax_; i++) {
      if (filterState.fsm_.isValid_[i]) {
        FeatureManager<mtState::nLevels_, mtState::patchSize_, mtState::nCam_>& f = filterState.fsm_.features_[i];
        if (!f.mpStatistics_->isGoodFeature(trackingUpperBound_, trackingLowerBound_)) {
          if (verbose_) std::cout << filterState.fsm_.features_[i].idx_ << ", ";
          filterState.fsm_.isValid_[i] = false;
          filterState.resetFeatureCovariance(i, Eigen::Matrix3d::Identity());
        }
      }
    }
    if (verbose_) std::cout << " | ";
    // Check if enough free features, enforce removal
    int requiredFreeFeature = mtState::nMax_ * minTrackedAndFreeFeatures_ - countTracked;
    double factor = removalFactor_;
    int featureIndex = 0;
    while ((int)(mtState::nMax_) - (int)(filterState.fsm_.getValidCount()) < requiredFreeFeature) {
      if (filterState.fsm_.isValid_[featureIndex]) {
        FeatureManager<mtState::nLevels_, mtState::patchSize_, mtState::nCam_>& f = filterState.fsm_.features_[featureIndex];
        if (!f.mpStatistics_->trackedInSomeFrame() && !f.mpStatistics_->isGoodFeature(trackingUpperBound_ * factor, trackingLowerBound_ * factor)) {
          if (verbose_) std::cout << filterState.fsm_.features_[featureIndex].idx_ << ", ";
          filterState.fsm_.isValid_[featureIndex] = false;
          filterState.resetFeatureCovariance(featureIndex, Eigen::Matrix3d::Identity());
        }
      }
      featureIndex++;
      if (featureIndex == mtState::nMax_) {
        featureIndex = 0;
        factor = factor * removalFactor_;
      }
    }
    if (verbose_) std::cout << std::endl;

    // Get new features
    if (filterState.fsm_.getValidCount() < startDetectionTh_ * mtState::nMax_) {
      // Compute the median depth parameters for each camera, using the state features.
      std::array<double, mtState::nCam_> medianDepthParameters;
      if (maxUncertaintyToDepthRatioForDepthInitialization_ > 0) {
        filterState.getMedianDepthParameters(initDepth_, &medianDepthParameters, maxUncertaintyToDepthRatioForDepthInitialization_);
      } else {
        medianDepthParameters.fill(initDepth_);
      }
      for (int camID = 0; camID < mtState::nCam_; camID++) {
        // Get Candidates
        if (verbose_) std::cout << "Adding keypoints" << std::endl;
        const double t1 = (double)cv::getTickCount();
        candidates_.clear();

        //CUSTOMIZATION
        //Histogram Equalization should only applied be if image is in 16-bit range
        bool applyHistogramEqualization = false;
        double imgMin, imgMax;
        cv::minMaxLoc(meas.aux().pyr_[camID].imgs_[startLevel_], &imgMin, &imgMax);
        if (imgMax > max_8bit_image_val)
          applyHistogramEqualization = true;
        if (verbose_)
          std::cout << "Image Min/Max:" << imgMin << "/" << imgMax << "\t applyHistogramEqualization=" << applyHistogramEqualization << std::endl;

        //Detect FAST corners, equalize image if 16-bit before application of OpenCV FAST detector
        for (int l = endLevel_; l <= startLevel_; l++) {
          meas.aux().pyr_[camID].detectFastCorners(candidates_,
                                                   l,
                                                   fastDetectionThreshold_,
                                                   mpMultiCamera_->cameras_[camID].valid_radius_,
                                                   applyHistogramEqualization);
        }
        //CUSTOMIZATION

        const double t2 = (double)cv::getTickCount();
        if (verbose_) std::cout << "== Detected " << candidates_.size() << " on levels " << endLevel_ << "-" << startLevel_ << " (" << (t2 - t1) / cv::getTickFrequency() * 1000 << " ms)" << std::endl;
        std::unordered_set<unsigned int> newSet = filterState.fsm_.addBestCandidates(candidates_,
                                                                                     meas.aux().pyr_[camID],
                                                                                     camID,
                                                                                     filterState.t_,
                                                                                     endLevel_,
                                                                                     startLevel_,
                                                                                     (mtState::nMax_ - filterState.fsm_.getValidCount()) / (mtState::nCam_ - camID),
                                                                                     nDetectionBuckets_,
                                                                                     scoreDetectionExponent_,
                                                                                     penaltyDistance_,
                                                                                     zeroDistancePenalty_,
                                                                                     false,
                                                                                     minAbsoluteSTScore_);
        const double t3 = (double)cv::getTickCount();
        if (verbose_) std::cout << "== Got " << filterState.fsm_.getValidCount() << " after adding " << newSet.size() << " features in camera " << camID << " (" << (t3 - t2) / cv::getTickFrequency() * 1000 << " ms)" << std::endl;

        //Add New Features
        for (auto it = newSet.begin(); it != newSet.end(); ++it) {
          FeatureManager<mtState::nLevels_, mtState::patchSize_, mtState::nCam_>& f = filterState.fsm_.features_[*it];
          f.mpStatistics_->resetStatistics(filterState.t_);
          f.mpStatistics_->status_[camID] = TRACKED;
          f.mpStatistics_->lastPatchUpdate_ = filterState.t_;
          const double initRelDepthCovTemp_ = initCovFeature_(0, 0);

          //CUSTOMIZATION - Attempt Direct Depth Initialization if LiDAR is available
          double feaCovScaleFactor = 1.0;
          f.mpCoordinates_->directDepthInit_ = false;
          if (useDepthFromLiDAR_ && !projectedDepthImgs_[camID].empty()) {
            float directDepth = getFeatureDirectDepth(f.mpCoordinates_->get_c(), projectedDepthImgs_[camID], *it);
            if (directDepth > 0) {
              f.mpDistance_->p_ = 1.0 / directDepth;
              f.mpCoordinates_->directDepthInit_ = true;
              f.mpStatistics_->lastDepthUpdate_ = filterState.t_;
              feaCovScaleFactor = directDepInitCovScaleFactor_;
            }
          }

          // If direct depth not available or fails use median feature depth
          if (!f.mpCoordinates_->directDepthInit_) {
            f.mpDistance_->p_ = medianDepthParameters[camID];
            feaCovScaleFactor = 1.0;
          }

          //Only Visualization - Display Feature IDs instead of TotalFeatureCount - Better for Debugging
          f.idx_ = *it;
          //CUSTOMIZATION

          initCovFeature_(0, 0) = initRelDepthCovTemp_ * std::pow(f.mpDistance_->getParameterDerivative() * f.mpDistance_->getDistance(), 2) * feaCovScaleFactor;  //CUSTOMIZATION
          filterState.resetFeatureCovariance(*it, initCovFeature_);
          initCovFeature_(0, 0) = initRelDepthCovTemp_;
          if (doFrameVisualisation_) {
            f.mpCoordinates_->drawPoint(filterState.img_[camID], cv::Scalar(255, 0, 0));
            f.mpCoordinates_->drawText(filterState.img_[camID], std::to_string(f.idx_), cv::Scalar(255, 0, 0));
          }

          if (mtState::nCam_ > 1 && doStereoInitialization_) {
            const int otherCam = (camID + 1) % mtState::nCam_;
            transformFeatureOutputCT_.setFeatureID(*it);
            transformFeatureOutputCT_.setOutputCameraID(otherCam);
            transformFeatureOutputCT_.transformState(filterState.state_, featureOutput_);
            if (alignment_.align2DAdaptive(alignedCoordinates_, meas.aux().pyr_[otherCam], *f.mpMultilevelPatch_, featureOutput_.c(), startLevel_, endLevel_,
                                           alignConvergencePixelRange_, alignCoverageRatio_, alignMaxUniSample_)) {
              bool valid = mlpTemp1_.isMultilevelPatchInFrame(meas.aux().pyr_[otherCam], alignedCoordinates_, startLevel_, false);
              if (valid && patchRejectionTh_ >= 0) {
                mlpTemp1_.extractMultilevelPatchFromImage(meas.aux().pyr_[otherCam], alignedCoordinates_, startLevel_, false);
                const float avgError = mlpTemp1_.computeAverageDifference(*f.mpMultilevelPatch_, endLevel_, startLevel_);
                if (avgError > patchRejectionTh_) {
                  valid = false;
                }
              }
              if (valid == true) {
                if (doFrameVisualisation_) {
                  alignedCoordinates_.drawPoint(filterState.img_[otherCam], cv::Scalar(150, 0, 0));
                  alignedCoordinates_.drawText(filterState.img_[otherCam], std::to_string(f.idx_), cv::Scalar(150, 0, 0));
                }
                if (f.mpCoordinates_->getDepthFromTriangulation(alignedCoordinates_, state.qCM(otherCam).rotate(V3D(state.MrMC(camID) - state.MrMC(otherCam))), state.qCM(otherCam) * state.qCM(camID).inverted(), *f.mpDistance_, 0.01)) {
                  filterState.resetFeatureCovariance(*it, initCovFeature_);  // TODO: improve
                }
              } else {
                if (doFrameVisualisation_) {
                  alignedCoordinates_.drawPoint(filterState.img_[otherCam], cv::Scalar(0, 0, 150));
                  alignedCoordinates_.drawText(filterState.img_[otherCam], std::to_string(f.idx_), cv::Scalar(0, 0, 150));
                }
              }
            } else {
              if (doFrameVisualisation_) {
                alignedCoordinates_.drawPoint(filterState.img_[otherCam], cv::Scalar(0, 150, 0));
                alignedCoordinates_.drawText(filterState.img_[otherCam], std::to_string(f.idx_), cv::Scalar(0, 150, 0));
              }
            }
          }
        }
      }
    }

    //CUSTOMIZATION - Check All Features for delayed direct depth intialization OR periodic depth update
    std::clock_t depthUpdateLoop_t = std::clock();
    for (int camID = 0; camID < mtState::nCam_; ++camID) {
      if (useDepthFromLiDAR_ && !projectedDepthImgs_[camID].empty()) {
        for (unsigned int ID = 0; ID < mtState::nMax_; ++ID) {
          //Get Feature
          auto& f = filterState.fsm_.features_[ID];
          //Check 1.Feature is Valid AND 2.Either new depth initialization OR update depth after min time
          if (filterState.fsm_.isValid_[ID] &&
              (!f.mpCoordinates_->directDepthInit_ ||
               (filterState.t_ - f.mpStatistics_->lastDepthUpdate_) > minTimeBeforeFeaDistCheck_)) {
            //Get Feature Depth
            float directDepth = getFeatureDirectDepth(f.mpCoordinates_->get_c(), projectedDepthImgs_[camID], ID);
            if (directDepth > 0) {
              //Update Depth and status
              f.mpDistance_->p_ = 1.0 / directDepth;
              f.mpCoordinates_->directDepthInit_ = true;
              f.mpStatistics_->lastDepthUpdate_ = filterState.t_;

              //Get Old Covariance
              transformFeatureOutputCT_.setFeatureID(ID);
              transformFeatureOutputCT_.setOutputCameraID(camID);
              transformFeatureOutputCT_.transformState(filterState.state_, featureOutput_);
              MXD CovMat;
              transformFeatureOutputCT_.transformCovMat(filterState.state_, filterState.cov_, CovMat);

              // //DEBUG
              // std::cout << "Depth Update for Feature ID: " << ID
              //           << ", Depth: " << directDepth
              //           << ", CovMat Before:\n"
              //           << CovMat;
              // //DEBUG

              //Copy Pixel Covariance block
              M3D initCovMat = M3D::Zero();
              initCovMat.block<2, 2>(1, 1) = CovMat.block<2, 2>(0, 0);  // Pixel-Cov Block
              initCovMat(0, 0) = initCovFeature_(0, 0) * std::pow(f.mpDistance_->getParameterDerivative() * f.mpDistance_->getDistance(), 2) * directDepInitCovScaleFactor_;
              //Reset Feature Covaraince
              filterState.resetFeatureCovariance(ID, initCovMat);

              // //DEBUG
              // transformFeatureOutputCT_.transformCovMat(filterState.state_, filterState.cov_, CovMat);
              // std::cout << "\nAFTER:\n"
              //           << CovMat << std::endl;
              // //DEBUG
            }
          }
        }
      }
      if (verbose_) std::cout << "--- Timing - depthUpdateLoop: " << double(std::clock() - depthUpdateLoop_t) / CLOCKS_PER_SEC * 1000.0 << " ms" << std::endl;
    }
    //CUSTOMIZATION

    for (unsigned int i = 0; i < mtState::nMax_; i++) {
      if (filterState.fsm_.isValid_[i]) {
        filterState.fsm_.features_[i].log_previous_ = *filterState.fsm_.features_[i].mpCoordinates_;
      }
    }

    if (doFrameVisualisation_) {
      for (int i = 0; i < mtState::nCam_; i++) {
        drawVirtualHorizon(filterState, i);
      }
    }

    if (verbose_) {
      for (int i = 0; i < mtState::nCam_; i++) {
        std::cout << "Camera extrinsics: " << i << std::endl;
        std::cout << "  " << filterState.state_.qCM(i) << std::endl;
        std::cout << "  " << filterState.state_.MrMC(i).transpose() << std::endl;
      }
    }

    // Copy image pyramid to state
    for (int i = 0; i < mtState::nCam_; i++) {
      filterState.prevPyr_[i] = meas.aux().pyr_[i];
    }

    // Zero Velocity updates if appropriate
    if (isZeroVelocityUpdateEnabled_ && doVisualMotionDetection_ && filterState.state_.aux().timeSinceLastImageMotion_ > minTimeForZeroVelocityUpdate_ && filterState.state_.aux().timeSinceLastInertialMotion_ > minTimeForZeroVelocityUpdate_) {
      cv::putText(filterState.img_[0], "Performing Zero Velocity Updates!", cv::Point2f(150, 25), cv::FONT_HERSHEY_SIMPLEX, 1.0, cv::Scalar(0, 255, 255));
      zeroVelocityUpdate_.performUpdateEKF(filterState, ZeroVelocityUpdateMeas<mtState>());
    }
  }
  ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

  /** \brief Draws a virtual horizon into the current image of the camera with ID camID
   *
   *  @param filterState - Filter state.
   *  @param camID       - ID of the camera, in which image the horizon should be drawn.
   */
  void drawVirtualHorizon(mtFilterState& filterState, const int camID = 0) {
    typename mtFilterState::mtState& state = filterState.state_;
    cv::rectangle(filterState.img_[camID], cv::Point2f(0, 0), cv::Point2f(82, 92), cv::Scalar(50, 50, 50), -1, 8, 0);
    cv::rectangle(filterState.img_[camID], cv::Point2f(0, 0), cv::Point2f(80, 90), cv::Scalar(100, 100, 100), -1, 8, 0);
    cv::putText(filterState.img_[camID], std::to_string(filterState.imageCounter_), cv::Point2f(5, 85), cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(255, 0, 0));
    cv::Point2f rollCenter = cv::Point2f(40, 40);
    cv::Scalar rollColor1(50, 50, 50);
    cv::Scalar rollColor2(200, 200, 200);
    cv::Scalar rollColor3(120, 120, 120);
    cv::circle(filterState.img_[camID], rollCenter, 32, rollColor1, -1, 8, 0);
    cv::circle(filterState.img_[camID], rollCenter, 30, rollColor2, -1, 8, 0);
    Eigen::Vector3d Vg = (state.qCM(camID) * state.qWM().inverted()).rotate(Eigen::Vector3d(0, 0, -1));
    double roll = atan2(Vg(1), Vg(0)) - 0.5 * M_PI;
    double pitch = acos(Vg.dot(Eigen::Vector3d(0, 0, 1))) - 0.5 * M_PI;
    double pixelFor10Pitch = 5.0;
    double pitchOffsetAngle = -asin(pitch / M_PI * 180.0 / 10.0 * pixelFor10Pitch / 30.0);
    cv::Point2f rollVector1 = 30 * cv::Point2f(cos(roll), sin(roll));
    cv::Point2f rollVector2 = cv::Point2f(25, 0);
    cv::Point2f rollVector3 = cv::Point2f(10, 0);
    std::vector<cv::Point> pts;
    cv::ellipse2Poly(rollCenter, cv::Size(30, 30), 0, (roll - pitchOffsetAngle) / M_PI * 180, (roll + pitchOffsetAngle) / M_PI * 180 + 180, 1, pts);
    cv::Point* points;
    points = &pts[0];
    int nbtab = pts.size();
    cv::fillPoly(filterState.img_[camID], (const cv::Point**)&points, &nbtab, 1, rollColor3);
    cv::line(filterState.img_[camID], rollCenter + rollVector2, rollCenter + rollVector3, rollColor1, 2);
    cv::line(filterState.img_[camID], rollCenter - rollVector2, rollCenter - rollVector3, rollColor1, 2);
    cv::ellipse(filterState.img_[camID], rollCenter, cv::Size(10, 10), 0, 0, 180, rollColor1, 2, 8, 0);
    cv::circle(filterState.img_[camID], rollCenter, 2, rollColor1, -1, 8, 0);
  }

  //CUSTOMIZATION
  /** \brief Project LiDAR points from LiDAR frame to Image frame of camera with ID camID
   *
   *  @param cloud - LiDAR pointcloud
   *  @param lidar2cameraTF - LiDAR-to-Camera transform (Obtained from camera2LiDAR calibration)
   *  @param camID - ID of the camera, in which image the LiDAR points should be projected.
   *  @param dImg - Depth Image created by projecting valid LiDAR points into camera frame
   */
  void projectPCLtoImgFrame(const pcl::PointCloud<pcl::PointXYZI>& cloud, const tf::Transform& lidar2cameraTF, const int& camID, cv::Mat& dImg) {
    //If point cloud is empty return an empty Mat
    if (cloud.points.size() == 0) {
      dImg = cv::Mat();
      return;
    }

    //Loop through LiDAR points
    float* matPtr = (float*)dImg.data;
    for (const auto& pt : cloud.points) {
      //Check if point is good
      if (!std::isfinite(pt.x) || !std::isfinite(pt.y) || !std::isfinite(pt.z))
        continue;

      //Ignore zero points
      if (pt.x == 0.0 && pt.y == 0.0 && pt.z == 0.0)
        continue;

      //Transform point from LiDAR frame to Camera frame and remove points behind camera
      tf::Vector3 pt_cam = lidar2cameraTF * tf::Vector3(pt.x, pt.y, pt.z);

      //Check if point is behind camera (ideally zero but 10cm to avoid noisy points)
      if (pt_cam.getZ() < 0.1)
        continue;

      //Project into Image frame
      cv::Point2f pxf;
      bool success = mpMultiCamera_->cameras_[camID].bearingToPixel(Eigen::Vector3d(pt_cam.getX(), pt_cam.getY(), pt_cam.getZ()), pxf);
      if (!success)
        continue;
      cv::Point2i px(cvRound(pxf.x), cvRound(pxf.y));

      //Assign Depth
      if (px.x > 0 && px.y > 0 && px.x < dImg.cols && px.y < dImg.rows) {
        //Calculate range w.r.t Camera Frame
        float range = std::sqrt(pt_cam.getX() * pt_cam.getX() + pt_cam.getY() * pt_cam.getY() + pt_cam.getZ() * pt_cam.getZ());

        //If not assigned i.e. 0 - assign depth
        if (matPtr[px.y * dImg.cols + px.x] == 0)
          matPtr[px.y * dImg.cols + px.x] = range;  //current row * number of coloumns offset + current coloumn
        else if (range < matPtr[px.y * dImg.cols + px.x])
          matPtr[px.y * dImg.cols + px.x] = range;  //If already assigned - re-assign nearest depth

        if (doFrameVisualisation_) {
          float red = (3.0 * range / maxAllowedFeatureDistance_) * 255.0;
          red = (red > 255) ? 255 : red;
          cv::circle(drawImg_, px, 1, cv::Scalar(0, (255 - (int)red), (int)red), -1);
        }
      }
    }
  }

  /** \brief Get LiDAR depth surrounding a camera feature from the depth image created by projectPCLtoImgFrame
   *
   *  @param ptf - Pixel Coordinates of camera feature
   *  @param depthImg - Depth Image for depth look-up
   *  @param feaID - ID of feature
   *  @param tol - Tolereance of Std.Dev of depth patch
   *  @return - Mean Patch Depth, -1 if invalid patch
   */
  float getFeatureDirectDepth(const cv::Point2i& ptf, const cv::Mat& depthImg, const int& feaID, float tol = 0.1) {
    //Define look-up window and center point
    static const int hWinHalf = depthSearchWindowWidth_ / 2;
    static const int vWinHalf = depthSearchWindowHeight_ / 2;

    //Check if look-up ROI is within image
    cv::Point2i pt(cvRound(ptf.x), cvRound(ptf.y));
    int uMin = pt.x - hWinHalf;
    int vMin = pt.y - vWinHalf;
    //Check if ROI boundaries are valid
    if (uMin < 0 || vMin < 0)
      return -1;
    if ((pt.x + hWinHalf) > depthImg.cols - 1 || (pt.y + vWinHalf) > depthImg.rows - 1)
      return -1;

    //Count Non-Zero depth points in the ROI
    cv::Rect ROI(uMin, vMin, depthSearchWindowWidth_, depthSearchWindowHeight_);
    cv::Mat depthPatch(depthImg, ROI);
    cv::Mat countPatch;
    cv::normalize(depthPatch, countPatch, 0, 255, cv::NORM_MINMAX, CV_8UC1);
    std::vector<cv::Point2i> ROInonZeroLoc;
    cv::findNonZero(countPatch, ROInonZeroLoc);

    //If not enough non-zero depth points neighbor present
    if (ROInonZeroLoc.size() < min_non_zero_neighbors_)
      return -1;

    //Get Depth
    float depth = -1;
    if (lidarDepthCalcMode_ == 0)
      depth = getDirectDepthFromPatch(depthPatch, ROInonZeroLoc, tol);
    else if (lidarDepthCalcMode_ == 1)
      depth = getDirectDepthFromLine(depthPatch, ROInonZeroLoc, tol);
    else
      ROS_WARN_STREAM("=== ROVIO - INVALID DEPTH CALCULATION MODE SET IT SHOUD BE EITHER 0(PATCH) or 1(LINE)===");

    if (depth > maxAllowedFeatureDistance_)
      return -1;

    //DEBUG
    // std::cout << "FeaID: " << feaID
    //           << ", MODE: " << (lidarDepthCalcMode_ == 0 ? "PATCH" : "LINE")
    //           << ", NonZero:" << ROInonZeroLoc.size()
    //           << ", depth: " << depth << std::endl;
    //DEBUG

    return depth;
  }

  /** \brief Get LiDAR depth using a Patch surrounding a camera feature
   *
   *  @param depthPatch - Patch containing depth values surrounding the feature location
   *  @param ROInonZeroLoc - Vector of non-zero depth locations in the depthPatch
   *  @param tol - Tolereance of Std.Dev of depth patch
   *  @return - Mean Patch Depth, -1 if invalid patch
   */
  float getDirectDepthFromPatch(const cv::Mat& depthPatch, const std::vector<cv::Point2i>& ROInonZeroLoc, float tol = 0.1) {
    //Loop though non-zero points
    static const cv::Point2i ctrPt(depthSearchWindowWidth_ / 2, depthSearchWindowHeight_ / 2);
    int topCount = 0, bottomCount = 0;
    //Calculate Mean Depth
    float meanDep = 0.0;
    for (auto& roiPt : ROInonZeroLoc) {
      meanDep += depthPatch.at<float>(roiPt);
      //Check if point is above or below the center
      if (roiPt.y < ctrPt.y)
        ++topCount;
      else
        ++bottomCount;
    }
    meanDep /= ROInonZeroLoc.size();

    // Single Beam - No information of planarity
    if (topCount < 2 || bottomCount < 2)
      return -1;

    //Calculate Std.Dev
    float stdDev = 0.0;
    for (auto& roiPt : ROInonZeroLoc)
      stdDev += (depthPatch.at<float>(roiPt) - meanDep) * (depthPatch.at<float>(roiPt) - meanDep);
    stdDev /= (ROInonZeroLoc.size() - 1);
    stdDev = std::sqrt(stdDev);

    //Check if patch has large Std.Dev
    if (stdDev > tol)
      return -1;

    return meanDep;
  }

  /** \brief Get LiDAR depth using a LINE nearest to the camera feature
   *
   *  @param depthPatch - Patch containing depth values surrounding the feature location
   *  @param ROInonZeroLoc - Vector of non-zero depth locations in the depthPatch
   *  @param tol - Tolereance of Max depth difference between consective points on the line
   *  @return - Mean Line Depth, -1 if invalid
   */
  float getDirectDepthFromLine(const cv::Mat& depthPatch, const std::vector<cv::Point2i>& ROInonZeroLoc, float tol = 0.1) {
    //Loop though non-zero points
    static const cv::Point2i ctrPt(depthSearchWindowWidth_ / 2, depthSearchWindowHeight_ / 2);
    int leftCount = 0, rightCount = 0;
    //Calculate Mean Depth
    float meanDep = 0.0;
    for (auto& roiPt : ROInonZeroLoc) {
      meanDep += depthPatch.at<float>(roiPt);
      //Check if point is above or below the center
      if (roiPt.x < ctrPt.x)
        ++leftCount;
      else
        ++rightCount;
    }
    meanDep /= ROInonZeroLoc.size();

    // Single Beam - Balanced Neighborhoods
    if (leftCount < 2 || rightCount < 2)
      return -1;

    //Calculate Max depth difference between consective points
    float maxDepDiff = 0.0;
    for (int i = 1; i < ROInonZeroLoc.size(); ++i) {
      float depDiff = std::abs(depthPatch.at<float>(ROInonZeroLoc[i]) - depthPatch.at<float>(ROInonZeroLoc[i - 1]));
      if (depDiff > maxDepDiff)
        maxDepDiff = depDiff;
    }

    //Check if line has large depth differences
    if (maxDepDiff > tol)
      return -1;

    return meanDep;
  }
  //CUSTOMIZATION

};  // namespace rovio

}  // namespace rovio
#endif /* ROVIO_IMGUPDATE_HPP_ */
