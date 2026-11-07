#!/bin/bash
# Build compslam_vtio inside the container for NCAM cameras (1 = RGB, 2 = RGB +
# thermal) into tartanstar/build/ws_ncam$NCAM. Only compslam_vtio is linked into
# the workspace: it builds its own copy of lightweight_filtering
# (add_subdirectory), and the compslam meta package needs a missing compslam_se.
set -e
NCAM=${NCAM:-1}
WS=/compslam_subt/tartanstar/build/ws_ncam$NCAM
mkdir -p "$WS/src"
ln -sfn /compslam_subt/compslam_vtio "$WS/src/compslam_vtio"
cd "$WS"
# OpenCV 4 keeps the legacy C constants (CV_GRAY2RGB, ...) in imgproc/types_c.h.
catkin_make -j"$(nproc)" -DCMAKE_BUILD_TYPE=Release -DMAKE_SCENE=OFF -DROVIO_NCAM="$NCAM" \
    -DCMAKE_CXX_FLAGS="-isystem /usr/include/opencv4 -include opencv2/imgproc/types_c.h"
