# Complementary SLAM

Main Author: Shehryar Khattak ([skhattak@ethz.ch](mailto:skhattak@ethz.ch))

Contact: Julian Nubert ([nubertj@ethz.ch](mailto:nubertj@ethz.ch))

This repository contains the complementary SLAM code.
It consists of the following packages:
1. *compslam*: Meta package that builds the entire codestack and can be used to launch the contained components.
2. *compslam_lio*: LiDAR Odometry package based on LOAM with improved robustness for field missions and the incorporation of IMU.
3. *compslam_vtio*: Visual-thermal odoemetry that builds on top of ROVIO. It also inccludes the possibility to use LiDAR for scale estimation as well as thermal camera support.
4. *compslam_se*: This is factor-graph-based state estimation package that currently lives [here](https://bitbucket.org/leggedrobotics/graph_msf_dev/).

## Compiling
Note that for your use-case you might only require one specific package, such as compslam_lio or graph_msf_dev.
To build the entire stack, execute:
```bash
catkin build compslam -DYMAKE_BUILD_TYPE=ReleaseWithDebInfo
```