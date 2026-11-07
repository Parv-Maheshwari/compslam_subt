#!/bin/bash
# Source ROS, kindr + stub, and the workspace built for NCAM cameras (default 1).
set -e
source /opt/ros/noetic/setup.bash
source /opt/deps/setup.bash
WS=/compslam_subt/tartanstar/build/ws_ncam${NCAM:-1}
[ -f "$WS/devel/setup.bash" ] && source "$WS/devel/setup.bash" --extend
exec "$@"
