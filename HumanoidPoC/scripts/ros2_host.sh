#!/usr/bin/env bash
# =============================================================================
# Host-side ros2 CLI wrapper (Isaac Sim Python 3.11 venv 用).
#
# IsaacSim-ros_workspaces は Python 3.11 でビルドされており、その rclpy C拡張は
# 3.11 でしかロードできない。一方ワークスペースの `ros2` スクリプトの shebang は
# `env python3` で、環境によってはシステムの Python 3.10 を拾って
#   ModuleNotFoundError: No module named 'rclpy._rclpy_pybind11'
# （cpython-310 の .so を探す）で失敗する。
#
# 本ラッパは venv の python(3.11) を絶対パスで使って ros2 を起動し、
# ワークスペース source / RMW / LD_LIBRARY_PATH(conda の libpython 等) を自動設定する。
#
# 使い方:  ./HumanoidPoC/scripts/ros2_host.sh topic list
#          ./HumanoidPoC/scripts/ros2_host.sh topic hz /lidar/points
# 上書き可:  ISAAC_PY / ISAAC_ROS_WS / RMW_IMPLEMENTATION / ROS_DOMAIN_ID
# =============================================================================
set -o pipefail

ISAAC_PY="${ISAAC_PY:-$HOME/env_isaaclab_2.2/bin/python}"
ISAAC_ROS_WS="${ISAAC_ROS_WS:-$HOME/IsaacSim-ros_workspaces/build_ws/humble}"
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_cyclonedds_cpp}"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"

for s in "$ISAAC_ROS_WS/humble_ws/install/setup.bash" \
         "$ISAAC_ROS_WS/isaac_sim_ros_ws/install/local_setup.bash"; do
  if [ -f "$s" ]; then
    # shellcheck disable=SC1090
    source "$s"
  else
    echo "[ros2_host] WARNING: ROS workspace not found: $s" >&2
  fi
done

# venv(--copies)の base_prefix = 元の conda 環境。その lib に libpython3.11 等があるので
# rclpy C拡張のロードのため LD_LIBRARY_PATH に追加する（conda 名に依存せず動的に解決）。
_BP="$("$ISAAC_PY" -c 'import sys; print(sys.base_prefix)' 2>/dev/null || true)"
if [ -n "$_BP" ] && [ -d "$_BP/lib" ]; then
  export LD_LIBRARY_PATH="$_BP/lib:${LD_LIBRARY_PATH:-}"
fi

ROS2_BIN="$(command -v ros2 || true)"
if [ -z "$ROS2_BIN" ]; then
  echo "[ros2_host] ERROR: ros2 が PATH に見つかりません（ワークスペース未ビルド/未source）" >&2
  exit 1
fi

exec "$ISAAC_PY" "$ROS2_BIN" "$@"
