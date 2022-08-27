# Complementrary SLAM LiDAR-Inertia Odometry

## GTSAM Installation

* Version: [GTSAM 4.1.1](https://github.com/borglab/gtsam/tree/4.1.1)
* Clone and Checkout:
```bash
git clone git@github.com:borglab/gtsam.git
cd gtsam
git checkout 4.1.1
mkdir build && cd build
```

### Local Install
* Use CMake with given options:
```bash
cmake -DCMAKE_INSTALL_PREFIX:PATH=$HOME/.local -DCMAKE_BUILD_TYPE=Release -DGTSAM_POSE3_EXPMAP=ON -DGTSAM_ROT3_EXPMAP=ON -DGTSAM_USE_QUATERNIONS=ON -DGTSAM_USE_SYSTEM_EIGEN=ON -DGTSAM_BUILD_WITH_MARCH_NATIVE=OFF ..
```
* Compile and install locally:
```bash
make install -j$(nproc)
```
* Environment variables (e.g. add to your .bashrc-file):
```
export CMAKE_PREFIX_PATH=$HOME/.local/:$CMAKE_PREFIX_PATH
export LD_LIBRARY_PATH=$HOME/.local/lib/:$LD_LIBRARY_PATH
export LIBRARY_PATH=${LIBRARY_PATH}:${LD_LIBRARY_PATH}
```

### Global Install
Execute the same cmake command as above, just without `-DCMAKE_INSTALL_PREFIX:PATH=$HOME/.local`.
Furthermore, you have to call
```bash
sudo make install -j$(nproc)
```

## Compilation
After GTSAM is installed correctly, you can simply compile *compslam_lio* using
```bash
catkin build compslam_lio -DCMAKE_BUILD_TYPE=ReleaseWithDebInfo
```