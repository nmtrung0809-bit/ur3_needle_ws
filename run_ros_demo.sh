#!/usr/bin/env bash
# Launch ROS demo (world uses controller <extern>).
# Critical on this host: conda puts python3.13 first; Jazzy rclpy needs python3.12.
set -eo pipefail
# Note: do NOT use `set -u` around ROS setup.bash — it references unset
# vars like AMENT_TRACE_SETUP_FILES and aborts with nounset.
ROOT="$(cd "$(dirname "$0")" && pwd)"

# Prefer system Python 3.12 over miniconda 3.13 (fixes rclpy._rclpy_pybind11)
export PATH="/usr/bin:/bin:${PATH}"
hash -r 2>/dev/null || true

# Webots extern IPC falls back oddly if USER unset
export USER="${USER:-abc}"
export USERNAME="${USERNAME:-$USER}"

# ROS setup scripts are not nounset-safe
set +u
# shellcheck disable=SC1091
source /opt/ros/jazzy/setup.bash
set -u

export WEBOTS_HOME="${WEBOTS_HOME:-/usr/local/webots}"
export ROS2_WEBOTS_HOME="${ROS2_WEBOTS_HOME:-$WEBOTS_HOME}"

cd "$ROOT"

# Stop previous Webots (basename only — avoid pgrep -f self-match)
pkill -x webots-bin 2>/dev/null || true
pkill -x webots 2>/dev/null || true
sleep 1
pkill -9 -x webots-bin 2>/dev/null || true
sleep 0.5
# Stale IPC dirs break Ros2Supervisor extern attach across USER names
rm -rf /tmp/webots/default /tmp/webots/abc 2>/dev/null || true

echo "[run_ros_demo] python3 -> $(command -v python3) ($(/usr/bin/python3 --version 2>&1))"
colcon build --packages-select ur3_needle_sim

set +u
# shellcheck disable=SC1091
source "$ROOT/install/setup.bash"
set -u

exec ros2 launch ur3_needle_sim demo.launch.py "$@"
