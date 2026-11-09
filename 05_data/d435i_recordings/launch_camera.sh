#!/usr/bin/env bash
# Use the RGB-D settings already verified on this machine.
set -eo pipefail
source /opt/ros/noetic/setup.bash
recording_dir=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
python3 "$recording_dir/upright_preview.py" &
preview_pid=$!
cleanup() {
  kill "$preview_pid" 2>/dev/null || true
  wait "$preview_pid" 2>/dev/null || true
}
trap cleanup EXIT
trap 'exit 130' INT
trap 'exit 143' TERM
roslaunch realsense2_camera rs_camera.launch \
  enable_depth:=true \
  enable_color:=true \
  enable_infra:=false \
  enable_infra1:=false \
  enable_infra2:=false \
  enable_gyro:=false \
  enable_accel:=false \
  depth_width:=640 \
  depth_height:=480 \
  depth_fps:=30 \
  color_width:=640 \
  color_height:=480 \
  color_fps:=30
