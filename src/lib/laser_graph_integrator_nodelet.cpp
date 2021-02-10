#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>
#include "nodelet/nodelet.h"

#include "loam/GraphIntegrator.hpp"

class LaserGraphIntegratorNodelet : public nodelet::Nodelet {
 public:
  LaserGraphIntegratorNodelet() {}
  ~LaserGraphIntegratorNodelet() {}

 private:
  //Function
  virtual void onInit() {
    //Get Node Handles
    nh_ = getMTNodeHandle();
    pnh_ = getMTPrivateNodeHandle();

    //Create Instance & Initialize
    graphInt_ = std::make_unique<loam::GraphIntegrator>();
    if (graphInt_->setup(nh_, pnh_))
      NODELET_WARN("--- Laser Graph Integrator Nodelet Initialized ---");
  }
  //Data
  ros::NodeHandle nh_, pnh_;
  std::unique_ptr<loam::GraphIntegrator> graphInt_;
};

//Declare as a Plug-in
PLUGINLIB_EXPORT_CLASS(LaserGraphIntegratorNodelet, nodelet::Nodelet);