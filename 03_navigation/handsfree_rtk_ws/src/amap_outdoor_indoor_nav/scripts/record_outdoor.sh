#!/usr/bin/env bash
#
# 室外数据集录包脚本：RTK + Super-LIO + far_planner 全链路。
#
# 用法：
#   ./record_outdoor.sh                 # 核心话题，录到 Ctrl-C
#   ./record_outdoor.sh --planning      # 额外录 far_planner / 地形图话题
#   ./record_outdoor.sh --no-cloud      # 不录 /lio/cloud_world（体积小很多）
#   ./record_outdoor.sh --duration 300  # 录 5 分钟后自动停
#   ./record_outdoor.sh --check-only    # 只做起飞前检查，不录
#
set -uo pipefail

OUT_DIR="${HOME}/Go2W_Project/05_data/rosbags"
PREFIX="outdoor"
DURATION=0
MAX_SIZE=0
RECORD_PLANNING=0
RECORD_CLOUD=1
CHECK_ONLY=0

# 对齐和复现的最小集合：没有 /tf 和 lio odom，回放时坐标关系全丢
CORE_TOPICS=(
  /handsfree/rtk/raw
  /handsfree/rtk/gnss
  /amap_nav/enu_pose
  /amap_nav/enu_origin
  /amap_nav/geo_status
  /amap_nav/gcj02_fix
  /lio/odom
  /lio/global_map
  /tf
  /tf_static
)

# 只在对齐阶段需要：室内外地图拼接、轨迹对齐都靠它
CLOUD_TOPICS=(
  /lio/cloud_world
)

# 需要回放 far_planner 行为时才录
PLANNING_TOPICS=(
  /scan_cloud
  /terrain_map
  /terrain_map_ext
  /terrain_cloud
  /terrain_local_cloud
  /odom_world
  /state_estimation
  /registered_scan
  /goal_point
  /way_point
  /far_reach_goal_status
  /viz_path_topic
  /navigation_boundary
  /cmd_vel
)

usage() {
  sed -n '2,11p' "$0" | sed 's/^# \{0,1\}//'
  exit 0
}

while [ $# -gt 0 ]; do
  case "$1" in
    -o|--output-dir) OUT_DIR="$2"; shift 2 ;;
    -p|--prefix)     PREFIX="$2"; shift 2 ;;
    --duration)      DURATION="$2"; shift 2 ;;
    --max-size)      MAX_SIZE="$2"; shift 2 ;;
    --planning)      RECORD_PLANNING=1; shift ;;
    --no-cloud)      RECORD_CLOUD=0; shift ;;
    --check-only)    CHECK_ONLY=1; shift ;;
    -h|--help)       usage ;;
    *) echo "未知参数: $1" >&2; exit 2 ;;
  esac
done

if ! command -v rosbag >/dev/null 2>&1; then
  echo "找不到 rosbag，请先 source ROS 环境（/opt/ros/noetic/setup.bash 和工作区 setup.bash）" >&2
  exit 1
fi

if ! timeout 5 rostopic list >/dev/null 2>&1; then
  echo "连不上 ROS master，请先启动 roscore 和导航栈" >&2
  exit 1
fi

TOPICS=("${CORE_TOPICS[@]}")
[ "$RECORD_CLOUD" -eq 1 ] && TOPICS+=("${CLOUD_TOPICS[@]}")
[ "$RECORD_PLANNING" -eq 1 ] && TOPICS+=("${PLANNING_TOPICS[@]}")

# ---------------------------------------------------------------- 起飞前检查
AVAILABLE="$(timeout 10 rostopic list 2>/dev/null)"
missing=0
for topic in "${TOPICS[@]}"; do
  if ! grep -qx "${topic}" <<<"${AVAILABLE}"; then
    echo "  [缺失] ${topic}"
    missing=$((missing + 1))
  fi
done

echo "计划录制 ${#TOPICS[@]} 个话题，缺失 ${missing} 个（缺失的会被 rosbag 跳过）"

if ! grep -qx "/amap_nav/enu_pose" <<<"${AVAILABLE}"; then
  echo "  [提示] 没有 /amap_nav/enu_pose：geo_coordinate_layer 未启动，或当前没有通过质量门限的定位" >&2
elif ! timeout 3 rostopic echo -n1 /amap_nav/enu_pose >/dev/null 2>&1; then
  echo "  [提示] /amap_nav/enu_pose 存在但没有数据：检查卫星数（GGA 第 8 位）和 min_gga_quality" >&2
fi

if ! grep -qx "/lio/odom" <<<"${AVAILABLE}"; then
  echo "  [提示] 没有 /lio/odom：Super-LIO 未启动，对齐节点无法工作" >&2
fi

if [ "$CHECK_ONLY" -eq 1 ]; then
  echo "检查完成（--check-only，未录制）"
  exit 0
fi

# -------------------------------------------------------------------- 录制
mkdir -p "${OUT_DIR}"
STAMP="$(date +%Y%m%d_%H%M%S)"
BAG="${OUT_DIR}/${PREFIX}_${STAMP}.bag"

ARGS=(-O "${BAG}" --lz4 -b 2048)
[ "$DURATION" -gt 0 ] && ARGS+=(--duration "${DURATION}")
[ "$MAX_SIZE" -gt 0 ] && ARGS+=(--split --size "${MAX_SIZE}")

echo "输出: ${BAG}"
echo "开始录制，Ctrl-C 结束..."

rosbag record "${ARGS[@]}" "${TOPICS[@]}"

# -------------------------------------------------------------------- 收尾
if [ -f "${BAG}" ]; then
  echo
  echo "===== 录制结果 ====="
  rosbag info "${BAG}" 2>/dev/null | sed -n '1,12p'
  echo
  echo "回放:  rosbag play ${BAG} --clock"
  echo "对齐:  rosrun amap_outdoor_indoor_nav enu_lio_align.py"
else
  echo "没有生成 bag 文件" >&2
  exit 1
fi
