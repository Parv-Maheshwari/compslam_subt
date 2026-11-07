#!/bin/bash
# Start the container: repo -> /compslam_subt, BAG_DIR (dir holding the clip) -> /bags (ro).
# usage: [NCAM=1|2] BAG_DIR=/path/to/bags tartanstar/docker.sh [command...]
set -e
REPO=$(cd "$(dirname "$0")/.." && pwd)
BAG_DIR=${BAG_DIR:?set BAG_DIR to the directory holding the bag}
TTY=(-i); [ -t 0 ] && [ -t 1 ] && TTY=(-it)
[ $# -eq 0 ] && set -- bash
docker run --rm "${TTY[@]}" --net host --user "$(id -u):$(id -g)" \
    -e HOME=/tmp -e ROS_HOME=/tmp/.ros -e NCAM="${NCAM:-1}" \
    -v "$REPO":/compslam_subt -v "$(realpath "$BAG_DIR")":/bags:ro \
    -w /compslam_subt compslam_tartanstar:latest "$@"
