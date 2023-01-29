# Complementrary SLAM LiDAR-Inertia Odometry

## GTSAM Installation

* Version: [GTSAM on branch develop_rsl](https://github.com/leggedrobotics/gtsam_fork/tree/develop_rsl).
* This version has already the latest bug fixes of the CombinedImuFactor. For more information we refer to this [issue](https://github.com/borglab/gtsam/commit/587678e0b761188e33db376cd552c417650c96ee#diff-278a5de28881129d36f253ed7fa64129312e5e1d9a042aa850424c2a93392e1f).
* Clone and checkout at an arbitrary location.
```bash
git clone https://github.com/leggedrobotics/gtsam_fork
cd gtsam_fork
git checkout develop_rsl
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