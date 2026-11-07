# compslam_vtio build + runtime on ROS Noetic (Ubuntu 20.04). Upstream code is
# not modified; the image only supplies what the build expects (see README):
#   - kindr (catkin package) and an empty yaml_cpp_catkin stub;
#   - /usr/share/OpenCV -> focal's OpenCV 4 cmake dir (CMakeLists forces OpenCV_DIR);
#   - image_transport (included by feature_tracker_node, not declared).
# Source is mounted at /compslam_subt and built by tartanstar/build.sh.
FROM ros:noetic-ros-base-focal

ENV DEBIAN_FRONTEND=noninteractive
RUN apt-get update && apt-get install -y --no-install-recommends \
        build-essential git cmake pkg-config \
        libeigen3-dev libyaml-cpp-dev libopencv-dev libpcl-dev \
        python3-catkin-tools \
        ros-noetic-cmake-modules ros-noetic-cv-bridge ros-noetic-tf \
        ros-noetic-pcl-ros ros-noetic-pcl-conversions ros-noetic-image-transport \
        ros-noetic-rosbag ros-noetic-roslaunch ros-noetic-rospy \
        ros-noetic-message-generation ros-noetic-message-runtime \
        ros-noetic-geometry-msgs ros-noetic-sensor-msgs ros-noetic-nav-msgs \
        ros-noetic-std-msgs ros-noetic-roslib ros-noetic-visualization-msgs \
    && rm -rf /var/lib/apt/lists/*

RUN ln -s "$(dirname "$(find /usr/lib -name OpenCVConfig.cmake | head -n1)")" /usr/share/OpenCV

COPY yaml_cpp_catkin_stub /opt/deps_ws/src/yaml_cpp_catkin
RUN git clone --depth 1 --branch 1.2.0 https://github.com/ethz-asl/kindr.git /opt/deps_ws/src/kindr \
    && . /opt/ros/noetic/setup.sh \
    && cd /opt/deps_ws \
    && catkin_make_isolated --install --install-space /opt/deps \
        -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF --catkin-make-args install \
    && rm -rf /opt/deps_ws

COPY entrypoint.sh /entrypoint.sh
RUN chmod 755 /entrypoint.sh
ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]
