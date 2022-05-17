#include <ros/ros.h>

#include "compslam_lio/LaserOdometry.h"

/** Main node entry point. */
int main(int argc, char** argv) {
  ros::init(argc, argv, "laserOdometry");
  ros::NodeHandle node;
  ros::NodeHandle privateNode("~");

  //Create Instance & Initialize
  loam::LaserOdometry laserOdom(0.1);
  if (laserOdom.setup(node, privateNode))
    ros::spin();

  return 0;
}
