# README #

### Modification Info: ###
This repository builds on top of ROVIO (Robust Visual Inertial Odometry) open-source framework (BSD License), forked at the Commit 473223daa86fd98041515cf090d2a39692156a97.
The purpose of this fork is to add features to ROVIO and to extend it to work with thermal imagery and is referred to ROTIO (Robust Thermal Inertial Odometry) for identification and operational purposes. The code is open-source (BSD License) as in case of the original work of ROVIO.

### Original ROVIO README and Paper: ###

This repository contains the ROVIO (Robust Visual Inertial Odometry) framework. The code is open-source (BSD License). Please remember that it is strongly coupled to on-going research and thus some parts are not fully mature yet. Furthermore, the code will also be subject to changes in the future which could include greater re-factoring of some parts.

Video: https://youtu.be/ZMAISVy-6ao

Papers:
* http://dx.doi.org/10.3929/ethz-a-010566547 (IROS 2015)
* http://dx.doi.org/10.1177/0278364917728574 (IJRR 2017)

Please also have a look at the wiki: https://github.com/ethz-asl/rovio/wiki

### Information regarding modifications incorporated in ROTIO: ###

Video: https://youtu.be/SOJAYlGqYec

Related paper:
* https://ieeexplore.ieee.org/document/8798279 (ICUAS 2019)

### Install without opengl scene ###
Dependencies:  
* ros  
* kindr (https://github.com/ethz-asl/kindr)  
* **lightweight_filtering (as submodule, use "git submodule update --init --recursive")**  

Forced Library Versions:  
* **OpenCV version:** An explicit path for ROS(system) default version of OpenCV library is defined. This is to resolve conflict if multiple versions of OpenCV are installed e.g. version 3.2.0 installed by ROS Medlodic and version 3.4.2 installed by opencv3_catkin package (*used by alphasense driver and maplab*).  
* **Yaml-cpp:** Use of Yaml-cpp is forced even if yaml_catkin_cpp is installed as the later seems to be deprecated.  

```
#!command

catkin build rovio --cmake-args -DCMAKE_BUILD_TYPE=Release
```

### Install with opengl scene ###
Additional dependencies: opengl, glut, glew (sudo apt-get install freeglut3-dev, sudo apt-get install libglew-dev)
```
#!command

catkin build rovio --cmake-args -DCMAKE_BUILD_TYPE=Release -DMAKE_SCENE=ON
```

### Euroc Datasets ###
The rovio_node.launch file loads parameters such that ROVIO runs properly on the Euroc datasets. The datasets are available under:
http://projects.asl.ethz.ch/datasets/doku.php?id=kmavvisualinertialdatasets

### Further notes ###
* Camera matrix and distortion parameters should be provided by a yaml file or loaded through rosparam
* The cfg/rovio.info provides most parameters for rovio. The camera extrinsics qCM (quaternion from IMU to camera frame, Hamilton-convention) and MrMC (Translation between IMU and Camera expressed in the IMU frame) should also be set there. They are being estimated during runtime so only a rough guess should be sufficient.
* Especially for application with little motion fixing the IMU-camera extrinsics can be beneficial. This can be done by setting the parameter doVECalibration to false. Please be carefull that the overall robustness and accuracy can be very sensitive to bad extrinsic calibrations.
