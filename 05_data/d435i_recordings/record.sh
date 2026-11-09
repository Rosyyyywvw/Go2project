#!/usr/bin/env bash
# Record one scene; invoke with bash record.sh SCENE [SECONDS].
set -eo pipefail

usage() {
  echo "用法: bash record.sh 场景 [秒数，默认60]"
  echo "秒数填 0 表示不限时，按 Ctrl+C 结束并保存。"
  echo "场景: 00_test 01_pedestrian 02_bicycle 03_cone 04_barrier 05_mixed 06_background 07_entrance"
}

if [[ ${1:-} == --help || ${1:-} == -h ]]; then
  usage
  exit 0
fi
if [[ $# -lt 1 || $# -gt 2 ]]; then
  usage >&2
  exit 2
fi
scene=$1
duration=${2:-60}
case "$scene" in
  00_test|01_pedestrian|02_bicycle|03_cone|04_barrier|05_mixed|06_background|07_entrance) ;;
  *) echo "未知场景: $scene" >&2; usage >&2; exit 2 ;;
esac
if [[ ! $duration =~ ^(0|[1-9][0-9]*)$ ]]; then
  echo '秒数必须为 0（不限时）或正整数，例如 10 或 60。' >&2
  exit 2
fi

source /opt/ros/noetic/setup.bash
set -u
recording_dir=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
if ! topics=$(timeout 10s rostopic list 2>&1); then
  echo '无法连接 ROS master；请先启动相机，并确认当前终端的 ROS_MASTER_URI。' >&2
  printf '%s\n' "$topics" >&2
  exit 1
fi
for topic in /camera/color/image_raw /camera/color/camera_info; do
  if ! grep -Fxq -- "$topic" <<< "$topics"; then
    echo "缺少话题 $topic，请先启动相机。" >&2
    exit 1
  fi
done

mkdir -p "$recording_dir/bags"
stamp=$(TZ=Asia/Shanghai date +%Y%m%d_%H%M%S_%N)
output="$recording_dir/bags/${scene}_${stamp}.bag"
if [[ -e $output || -e $output.active ]]; then
  echo "目标文件已存在: $output" >&2
  exit 1
fi
record_options=()
if [[ $duration == 0 ]]; then
  echo '录制模式: 不限时，按 Ctrl+C 结束并保存。'
else
  record_options+=(--duration="$duration")
  printf '时长上限: %s 秒；按 Ctrl+C 可提前结束并保存。\n' "$duration"
fi
printf '场景: %s\n输出: %s\n' "$scene" "$output"
exec rosbag record "${record_options[@]}" -O "$output" \
  /camera/color/image_raw \
  /camera/color/camera_info
