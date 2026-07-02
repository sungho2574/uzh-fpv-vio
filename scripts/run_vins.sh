#!/bin/bash
# Run one VINS-Mono variant against indoor_forward_9_davis_with_gt.bag.
#
# Meant to run INSIDE a container built from vins_ws/VINS-Mono/docker/Dockerfile,
# with these bind mounts:
#   -v $(pwd)/data:/root/data:ro
#   -v $(pwd)/config:/root/config:ro
#   -v $(pwd)/output:/root/output
#
# Build (on the amd64 server, from the repo root):
#   docker build -t vins-mono -f vins_ws/VINS-Mono/docker/Dockerfile vins_ws/VINS-Mono
#
# Run a container with this script mounted in too:
#   docker run --rm -it \
#     -v $(pwd)/data:/root/data:ro \
#     -v $(pwd)/config:/root/config:ro \
#     -v $(pwd)/output:/root/output \
#     -v $(pwd)/scripts:/root/scripts:ro \
#     vins-mono bash
#   # then inside the container:
#   bash /root/scripts/run_vins.sh no_loop
#   bash /root/scripts/run_vins.sh loop
#
# Usage: run_vins.sh <no_loop|loop> [rate]
#   rate: rosbag play speed multiplier, default 1.0 (real time).
#         Lower it (e.g. 0.5) if the estimator can't keep up in real time.

set -euo pipefail

VARIANT="${1:?usage: run_vins.sh <no_loop|loop> [rate]}"
RATE="${2:-1.0}"
BAG=/root/data/indoor_forward_9_davis_with_gt.bag
CONFIG=/root/config/uzh_fpv_davis_${VARIANT}.yaml

if [[ "$VARIANT" != "no_loop" && "$VARIANT" != "loop" ]]; then
    echo "error: variant must be 'no_loop' or 'loop', got '$VARIANT'" >&2
    exit 1
fi
if [[ ! -f "$CONFIG" ]]; then
    echo "error: config not found: $CONFIG" >&2
    exit 1
fi
if [[ ! -f "$BAG" ]]; then
    echo "error: bag not found: $BAG" >&2
    exit 1
fi

mkdir -p /root/output/${VARIANT}/pose_graph
source /root/catkin_ws/devel/setup.bash

echo "=== starting roscore ==="
roscore &
ROSCORE_PID=$!
sleep 3

echo "=== launching vins_estimator (variant=$VARIANT, config=$CONFIG) ==="
roslaunch vins_estimator euroc.launch config_path:=$CONFIG &
LAUNCH_PID=$!
sleep 5

echo "=== playing bag at rate=$RATE ==="
rosbag play "$BAG" --topics /dvs/image_raw /dvs/imu --clock --rate "$RATE"

echo "=== bag playback finished, letting estimator flush (10s) ==="
sleep 10

echo "=== shutting down ==="
kill $LAUNCH_PID 2>/dev/null || true
sleep 2
kill $ROSCORE_PID 2>/dev/null || true
wait 2>/dev/null || true

echo "=== done. output in /root/output/${VARIANT}/ ==="
ls -la /root/output/${VARIANT}/
