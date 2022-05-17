#include <ros/ros.h>

#include "compslam_lio/MultiScanRegistration.h"

/** Main node entry point. */
int main(int argc, char **argv) {
  ros::init(argc, argv, "scanRegistration");
  ros::NodeHandle node;
  ros::NodeHandle privateNode("~");

  //Create Instance & Initialize
  loam::MultiScanRegistration multiScan;
  if (multiScan.setup(node, privateNode))
    ros::spin();

  return 0;
}
