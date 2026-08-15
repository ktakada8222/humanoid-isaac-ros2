#!/usr/bin/env bash
# =============================================================================
# Launch the G1 locomotion + ROS 2 publisher environment (Isaac Sim 5.0).
#
# This wrapper absorbs the environment setup that the simulator needs so the
# locomotion script can be started with a single command:
#
#   * sources the Isaac-Sim-compatible ROS 2 workspaces so the correct
#     (Python 3.11) rclpy is importable;
#   * selects the RMW implementation so topics interoperate with the SLAM /
#     localization containers (CycloneDDS by default);
#   * preloads the interpreter's own libexpat, which Isaac Sim's loader would
#     otherwise shadow with an incompatible system copy.
#
# All paths can be overridden via environment variables before invoking:
#   ISAAC_PY        python in the Isaac Sim virtualenv
#                   (default: $HOME/env_isaaclab_2.2/bin/python)
#   ISAAC_ROS_WS    built IsaacSim-ros_workspaces humble dir
#                   (default: $HOME/IsaacSim-ros_workspaces/build_ws/humble)
#   RMW_IMPLEMENTATION  (default: rmw_cyclonedds_cpp)
#   ROS_DOMAIN_ID       (default: 0)
#
# Usage:  ./HumanoidPoC/scripts/run_locomotion.sh [extra args for the py script]
# =============================================================================
set -o pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_DIR="$(cd "$SCRIPT_DIR/../.." && pwd)"   # .../humanoid-isaac-ros2

ISAAC_PY="${ISAAC_PY:-$HOME/env_isaaclab_2.2/bin/python}"
ISAAC_ROS_WS="${ISAAC_ROS_WS:-$HOME/IsaacSim-ros_workspaces/build_ws/humble}"
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_cyclonedds_cpp}"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"

# --- source the ROS 2 workspaces (provides the Python 3.11 rclpy) ----------
for s in "$ISAAC_ROS_WS/humble_ws/install/setup.bash" \
         "$ISAAC_ROS_WS/isaac_sim_ros_ws/install/local_setup.bash"; do
  if [ -f "$s" ]; then
    # shellcheck disable=SC1090
    source "$s"
  else
    echo "[run_locomotion] WARNING: ROS workspace not found: $s" >&2
  fi
done

# --- preload the interpreter's libexpat ------------------------------------
# Isaac Sim injects an LD_LIBRARY_PATH that can put an older system libexpat
# ahead of the one the interpreter was built against, breaking pyexpat
# (matplotlib -> isaaclab_tasks). Preload the interpreter's own copy.
if [ -z "${LD_PRELOAD:-}" ]; then
  BASE_PREFIX="$("$ISAAC_PY" -c 'import sys; print(sys.base_prefix)' 2>/dev/null || true)"
  if [ -n "$BASE_PREFIX" ] && [ -f "$BASE_PREFIX/lib/libexpat.so.1" ]; then
    export LD_PRELOAD="$BASE_PREFIX/lib/libexpat.so.1"
  fi
fi

# --- run -------------------------------------------------------------------
cd "$REPO_DIR"
exec "$ISAAC_PY" \
  HumanoidPoC/scripts/environments/locomotion/g1/onnx_locomotion_g1.py "$@"
