#!/bin/bash
# Play a bag at 1x into compslam_vtio and record /compslam_vtio/odometry.
# Runs inside the container (tartanstar/docker.sh), with the workspace for the
# mode's camera count sourced (NCAM=1 for mono, NCAM=2 for dual).
# usage: run.sh mono|dual /bags/<clip>.bag OUT_DIR
# Writes OUT_DIR/{odometry.tum (t x y z qx qy qz qw, T_vio_map_imu), node.log, summary.txt}.
set -euo pipefail
MODE=$1 BAG=$2 OUT=$3
HERE=$(cd "$(dirname "$0")" && pwd)
want=1; [ "$MODE" = dual ] && want=2
[ "${NCAM:-1}" = "$want" ] || { echo "mode $MODE needs NCAM=$want (got ${NCAM:-1})" >&2; exit 1; }
[ ! -e "$OUT/odometry.tum" ] || { echo "$OUT/odometry.tum exists" >&2; exit 1; }
mkdir -p "$OUT"
export ROS_MASTER_URI=http://127.0.0.1:$(python3 -c 'import socket; s=socket.socket(); s.bind(("",0)); print(s.getsockname()[1])')
export ROS_HOSTNAME=127.0.0.1 ROS_LOG_DIR=$OUT/ros_log
roscore -p "${ROS_MASTER_URI##*:}" > "$OUT/roscore.log" 2>&1 & CORE=$!
trap 'kill $REC $NODE 2>/dev/null; sleep 2; kill $CORE 2>/dev/null; wait 2>/dev/null' EXIT
until rostopic list >/dev/null 2>&1; do sleep 0.5; done
stdbuf -oL -eL roslaunch "$HERE/launch/$MODE.launch" > "$OUT/node.log" 2>&1 & NODE=$!
# Recorder: one TUM line per odometry message (the node publishes only with a subscriber).
python3 - "$OUT/odometry.tum" <<'PY' & REC=$!
import sys, rospy
from nav_msgs.msg import Odometry
f = open(sys.argv[1], "w", buffering=1)
def cb(m):
    p, q = m.pose.pose.position, m.pose.pose.orientation
    f.write(f"{m.header.stamp.to_sec():.9f} {p.x:.6f} {p.y:.6f} {p.z:.6f} {q.x:.6f} {q.y:.6f} {q.z:.6f} {q.w:.6f}\n")
rospy.init_node("record_odometry"); rospy.Subscriber("/compslam_vtio/odometry", Odometry, cb, queue_size=10000)
rospy.spin()
PY
# Wait until the node subscribes to /imu and the recorder to the odometry.
until rostopic info /imu 2>/dev/null | grep -q compslam_vtio && \
      rostopic info /compslam_vtio/odometry 2>/dev/null | grep -q record_odometry; do sleep 0.5; done
rosbag play -q -d 2 "$BAG" > "$OUT/play.log" 2>&1
sleep 5   # let the node finish its queue
n=$(wc -l < "$OUT/odometry.tum")
{
  echo "mode: $MODE"
  echo "bag: $BAG"
  echo "poses: $n"
  [ "$n" -gt 0 ] && echo "first_last_stamp: $(head -n1 "$OUT/odometry.tum" | cut -d' ' -f1) $(tail -n1 "$OUT/odometry.tum" | cut -d' ' -f1)"
  [ "$n" -gt 0 ] && echo "last_position_xyz: $(tail -n1 "$OUT/odometry.tum" | cut -d' ' -f2-4)"
  [ "$n" -gt 0 ] && echo "path_length_m: $(awk 'NR>1{d+=sqrt(($2-x)^2+($3-y)^2+($4-z)^2)}{x=$2;y=$3;z=$4}END{printf "%.1f", d}' "$OUT/odometry.tum")"
  echo "filter_initialized: $(grep -c 'Filter: Initialized' "$OUT/node.log" || true)"
  echo "failed_synchronization_lines: $(grep -c 'Failed Synchronization' "$OUT/node.log" || true)"
} | tee "$OUT/summary.txt"
