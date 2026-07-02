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
#
# Also records two videos via scripts/record_video.py (needs to be mounted
# alongside this script, e.g. -v $(pwd)/scripts:/root/scripts:ro):
#   output/<variant>/vio_track.mp4     -- feature-tracked camera frames
#   output/<variant>/vio_track_3d.mp4  -- same, + a live 3D trajectory/attitude inset

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

echo "=== ensuring video recorder deps (matplotlib for python2) ==="
if ! python2 -c "import matplotlib" 2>/dev/null; then
    if ! command -v pip >/dev/null 2>&1; then
        apt-get update -qq && apt-get install -y -qq python-pip >/dev/null
    fi
    # The container's stock pip is ancient (Ubuntu 16.04) and doesn't honor
    # newer packages' python_requires metadata, so it happily fetches
    # py3-only releases of matplotlib's own deps (e.g. kiwisolver) and fails
    # to build them. Upgrade pip itself to the last version with Python 2
    # support first so dependency resolution picks compatible versions.
    pip install --quiet --upgrade "pip==20.3.4"
    pip install --quiet matplotlib==2.2.5 "numpy<1.17"
fi

echo "=== starting video recorder ==="
python2 /root/scripts/record_video.py --out-dir /root/output/${VARIANT} --fps 10 &
RECORDER_PID=$!
sleep 2

echo "=== playing bag at rate=$RATE ==="
rosbag play "$BAG" --topics /dvs/image_raw /dvs/imu --clock --rate "$RATE"

echo "=== bag playback finished, letting estimator flush (10s) ==="
sleep 10

echo "=== stopping video recorder ==="
kill -INT $RECORDER_PID 2>/dev/null || true
wait $RECORDER_PID 2>/dev/null || true

echo "=== shutting down ==="
kill $LAUNCH_PID 2>/dev/null || true
sleep 2
kill $ROSCORE_PID 2>/dev/null || true
wait 2>/dev/null || true

echo "=== done. output in /root/output/${VARIANT}/ ==="
ls -la /root/output/${VARIANT}/
