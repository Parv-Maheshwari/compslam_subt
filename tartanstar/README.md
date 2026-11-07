# compslam_vtio on TartanStar: minimal reproduction

This branch (`tartanstar`, based on upstream `85f1870`) adds only this
directory. **The upstream source is unmodified.** It shows:

- **RGB + IMU** (`ROVIO_NCAM=1`) runs and produces poses;
- **RGB + thermal + IMU** (`ROVIO_NCAM=2`) on the same clip runs without
  errors but never performs an image update: every image prints
  `Failed Synchronization of Camera Frames` and no pose is published.

**Question for the authors:** our RGB and thermal frames come from the same
30 Hz hardware trigger, but each is stamped at its own capture time, so the two
stamps of a pair differ by ~0.3 ms and are never identical. How should RGB and
thermal frames with non-identical stamps be paired in this code? For example:
was there a driver-side step that gave both cameras one shared stamp, a
tolerance we are missing, or a different input arrangement you used?

## TartanStar

TartanStar is a multi-modal dataset (CMU AirLab) with synchronized RGB,
thermal, LiDAR and inertial data recorded in indoor, outdoor, off-road and
night conditions, with LiDAR-based reference trajectories, used to benchmark
visual, thermal and multi-modal odometry. The
clip here is the first 60 s of the indoor trace `2026-05-25-12-20-25`, which
starts static.

## Sensors in the clip

| Topic | Sensor | Format | Rate |
|---|---|---|---|
| `/cam0/image_raw` | ZED X, left camera (RGB) | `mono8`, 960x600 (native 1920x1200 resized x0.5), distorted | 30 Hz |
| `/cam1/image_raw` | FLIR Boson+ thermal (left) | raw `mono16` 640x512, distorted; flat-field-correction (shutter) frames dropped | 30 Hz |
| `/imu` | MicroStrain GQ7 | rad/s, m/s^2, IMU frame; header seq consecutive | 100 Hz |

- Both cameras are **hardware triggered at 30 Hz** by one trigger signal.
- Camera stamps are each camera's corrected capture time on the GQ7 clock
  (camera-IMU offset 0). RGB and thermal stamps of the same trigger are **not
  identical**: nearest thermal minus RGB stamp is +0.34 ms median (p5
  -0.67 ms, p95 +0.34 ms); 0 of 1,770 RGB frames has a thermal frame with the
  identical stamp, 1,741 have one within 1 ms. We did not shift or snap any
  stamp.
- Calibration (`config/`): `cam0.yaml`, `cam1.yaml` are equidistant
  (Kannala-Brandt) intrinsics, the RGB one scaled to 960x600. The `.info`
  extrinsics: `qCM` = Hamilton (x, y, z, w) quaternion of R_cam_imu (IMU to
  camera), `MrMC` = camera origin in the IMU frame, from our URDF (the RGB one
  was verified against a ROVIO/ROVTIO run whose estimated trajectory axes
  match the reference).

Bag: `tartanstar_2026-05-25-12-20-25_first60s.bag` (ROS1, lz4, 1.7 GB,
1,770 + 1,743 images, 6,001 IMU messages), md5
`eef9bc7d82e5d182f4303ee28730786f`. Download: https://drive.google.com/drive/folders/1r0F40ePA_pI6IMWrDj875T-F30Vc2efi (folder with the bag, its `.md5` and `CONTENTS.md`).

## Steps

Requirements: Linux, Docker, an x86-64 CPU (upstream builds with
`-march=native`, so build on the machine you run on).

```bash
git clone --recursive -b tartanstar https://github.com/Parv-Maheshwari/compslam_subt.git
cd compslam_subt
export BAG_DIR=/path/to/dir/with/the/bag          # holds tartanstar_2026-05-25-12-20-25_first60s.bag
BAG=/bags/tartanstar_2026-05-25-12-20-25_first60s.bag

# (a) build: image, then one workspace per camera count
docker build -t compslam_tartanstar:latest tartanstar/docker
NCAM=1 tartanstar/docker.sh tartanstar/build.sh   # -> tartanstar/build/ws_ncam1
NCAM=2 tartanstar/docker.sh tartanstar/build.sh   # -> tartanstar/build/ws_ncam2

# (b) RGB + IMU: poses
NCAM=1 tartanstar/docker.sh tartanstar/run.sh mono $BAG tartanstar/runs/mono

# (c) RGB + thermal + IMU: Failed Synchronization, 0 poses
NCAM=2 tartanstar/docker.sh tartanstar/run.sh dual $BAG tartanstar/runs/dual
```

`run.sh` starts a roscore, launches `launch/<mode>.launch`, records
`/compslam_vtio/odometry` to `odometry.tum`, plays the bag at 1x real time
(~60 s), waits 5 s, and prints `summary.txt`. The node log is `node.log`.

## Expected output

Observed on a fresh clone of this branch (AMD Ryzen 9 5950X, Docker 20.04
image as above, 2026-11-07):

**(b) mono**, `tartanstar/runs/mono/summary.txt`:

```
mode: mono
bag: /bags/tartanstar_2026-05-25-12-20-25_first60s.bag
poses: 1767
first_last_stamp: 1779726026.150845289 1779726085.021454811
last_position_xyz: 7.878790 -44.693939 0.948581
path_length_m: 86.1
filter_initialized: 1
failed_synchronization_lines: 0
```

One pose per RGB image (1,770 in the bag), 58.9 s covered, no divergence.
For reference, against our LiDAR-based ground truth for this clip (68.5 m
path; evaluated outside this repo, evo `--align_origin`): ATE RMSE 6.6 m
(9.6% of the path), RPE over 10 m 15.5%. On the full 8 min trace the same mono
setup gives ATE 1.6% of 544 m and RPE over 10 m 12.3%.

**(c) dual**, `tartanstar/runs/dual/summary.txt`:

```
mode: dual
bag: /bags/tartanstar_2026-05-25-12-20-25_first60s.bag
poses: 0
filter_initialized: 1
failed_synchronization_lines: 3512
```

and `node.log`:

```
-- Filter: Initialized at t = 1779726025.15
    Failed Synchronization of Camera Frames, t = 1779726026.12
    Failed Synchronization of Camera Frames, t = 1779726026.15
    Failed Synchronization of Camera Frames, t = 1779726026.15
    ...
```

3,512 `Failed Synchronization` lines for 3,513 images (1,770 RGB + 1,743
thermal): every image except the first resets the image set, so no image
update runs and no pose is published (`odometry.tum` is empty). No errors, no
crash. (Consecutive lines show the same `t` because it is printed with 12
significant digits; the RGB and thermal stamps differ by ~0.3 ms.) Our full
8 min trace behaves the same: 28,587 lines for 28,588 images, 0 poses.

## Where it happens in the code (85f1870)

`compslam_vtio/include/compslam_vtio/RovioNode.hpp`, `imgCallback()`:

```cpp
double msgTime = img->header.stamp.toSec() + cam_offset;
if (msgTime != imgUpdateMeas_.template get<mtImgMeas::_aux>().imgTime_) {
  for (int i = 0; i < mtState::nCam_; i++) {
    if (imgUpdateMeas_.template get<mtImgMeas::_aux>().isValidPyr_[i]) {
      std::cout << "    \033[31mFailed Synchronization of Camera Frames, t = " << msgTime << "\033[0m" << std::endl;
    }
  }
  imgUpdateMeas_.template get<mtImgMeas::_aux>().reset(msgTime);
}
...
if (imgUpdateMeas_.template get<mtImgMeas::_aux>().areAllValid()) {
  mpFilter_->template addUpdateMeas<0>(imgUpdateMeas_, msgTime);
```

All cameras share one image set keyed by the exact time
`header.stamp + camN_offset`; an image with any other time resets the set,
and the update is queued only when every camera holds an image of exactly that
time (`areAllValid()`, `ImgUpdate.hpp`). With stamps ~0.3 ms apart the set
never completes. The filter initializes from the first IMU sample, but with no
image update its safe time never advances and nothing is published. A constant
`cam1_offset` cannot fix this here: the thermal - RGB offset is not constant
(+0.34 ms for most frames, -0.67 ms for some).

## Changes vs upstream (all in this directory)

| What | Upstream | Here | Why |
|---|---|---|---|
| Distro | ROVIO's Melodic / 18.04 setup | ROS Noetic / 20.04 Docker image | current ROS 1; same code builds |
| `OpenCV_DIR` forced to `/usr/share/OpenCV` | 18.04 path | image symlinks it to focal's OpenCV 4 cmake dir | no CMakeLists change |
| OpenCV legacy constants | OpenCV 3 headers | `-include opencv2/imgproc/types_c.h` in `CMAKE_CXX_FLAGS` | OpenCV 4 moved `CV_GRAY2RGB` etc. |
| `yaml_cpp_catkin` | declared dependency | empty stub catkin package in the image | CMakeLists already links the system yaml-cpp |
| `kindr` | from `repos.vcs` | ethz-asl/kindr 1.2.0 in the image | dependency |
| `image_transport` | not declared (used by `feature_tracker_node`) | installed in the image | build fails otherwise |
| Workspace | whole repo | only `compslam_vtio` (it builds its vendored `lightweight_filtering`) | `compslam` meta package needs a missing `compslam_se` |
| Build flags | `ROVIO_NCAM 1` | `ROVIO_NCAM` 1 or 2, `MAKE_SCENE=OFF`; `ROVIO_NMAXFEATURE` 25 (default) | mono / dual |
| `.info` Camera blocks | Alphasense | TartanStar (`config/compslam_vtio_{mono,dual}.info`) | our rig |
| `.info` `doFrameVisualisation` | true | false | headless |
| `.info` `useDepthFromLiDAR` | true | false | no LiDAR in the repro |
| `.info` all other values | upstream | unchanged | |
| Camera yamls | Alphasense 720x540 | `config/cam0.yaml` (RGB 960x600), `config/cam1.yaml` (thermal 640x512) | our cameras |
| Launch | `launch/compslam_vtio.launch` | `launch/{mono,dual}.launch`: our topics, `cam0_offset` / `cam1_offset` 0, frames without leading `/`, LiDAR topic unpublished, no compressed republish | our data |
