#!/usr/bin/env bash
# ARS: guarded teleoperation + lidar SLAM + camera 3D mapping, one command.
#
#   ./run_teleop_slam.sh                 full stack + RViz
#   ./run_teleop_slam.sh --no-camera     lidar only (camera unplugged)
#   ./run_teleop_slam.sh --no-slam       guarded teleop only, much lighter
#   ./run_teleop_slam.sh --no-rviz       no visualisation window
#   ./run_teleop_slam.sh --full-res      640x480 camera, but only 3.4 Hz mapping
#   ./run_teleop_slam.sh --bench         close-quarters thresholds for testing
#   ./run_teleop_slam.sh --wheel-odom    encoders instead of lidar ICP
#
# The stack runs in the background; the keyboard teleop runs in the foreground
# because it needs its own stdin. Ctrl-C stops teleop, and the trap then tears
# the whole stack down -- so the robot never keeps driving after you quit.
set -u

# Monitor mode puts the background job in its OWN process group, which is what
# makes the cleanup below able to kill the whole tree. Without it, killing the
# `ros2 launch` PID leaves every node child running -- and a stray
# serial_bridge holding /dev/arduino makes the Arduino look completely dead
# while still flashing fine, which is a genuinely confusing failure to chase.
set -m

ARGS=()
for a in "$@"; do
  case "$a" in
    --no-camera)     ARGS+=("use_camera:=false") ;;
    --no-slam)       ARGS+=("use_rtabmap:=false") ;;
    --no-rviz)       ARGS+=("use_rviz:=false") ;;
    --low-bandwidth) ARGS+=("low_bandwidth:=true") ;;   # now the default anyway
    --full-res)      ARGS+=("low_bandwidth:=false") ;;
    --wheel-odom)    ARGS+=("odom_source:=wheels") ;;
    --bench)         ARGS+=("stop_distance:=0.15" "slow_distance:=0.30") ;;
    *)               ARGS+=("$a") ;;
  esac
done

# ROS's setup files reference unset variables (AMENT_TRACE_SETUP_FILES and
# friends), so `set -u` aborts the moment they are sourced. Relax it just for
# the sourcing, then restore it for our own code where it is actually useful.
set +u
source /opt/ros/jazzy/setup.bash
source ~/ARS_Project/ydlidar_ros2_ws/install/setup.bash
source ~/ARS_Project/ars_ws/install/setup.bash
set -u

# ---- pre-flight: a leftover process on the serial port is the single most
# ---- confusing failure mode, so catch it before launching anything.
if command -v lsof >/dev/null 2>&1 && lsof /dev/ttyUSB0 >/dev/null 2>&1; then
  echo "WARNING: something already has /dev/ttyUSB0 open:"
  lsof /dev/ttyUSB0 2>/dev/null | tail -n +2 | sed 's/^/   /'
  echo "   The bridge will fail to open the Arduino. Clear it with:"
  echo "     pkill -9 -f 'install/ars_base/lib'"
  echo
fi

LOG=/tmp/ars_teleop_slam.log
echo "starting stack (log: $LOG)"
ros2 launch ars_base teleop_slam.launch.py "${ARGS[@]}" > "$LOG" 2>&1 &
STACK=$!

cleanup() {
  trap - EXIT INT TERM
  echo
  echo "stopping..."
  # Explicit zero first, rather than relying on the firmware watchdog to
  # time out.
  ros2 topic pub --once /cmd_vel_raw geometry_msgs/msg/Twist '{}' >/dev/null 2>&1 || true
  # Negative PID = whole process group. This is what actually reaps the nodes.
  kill -TERM -"$STACK" 2>/dev/null
  sleep 2
  kill -KILL -"$STACK" 2>/dev/null
  wait "$STACK" 2>/dev/null
  # Belt and braces: anything that escaped the group.
  pkill -9 -f "install/ars_base/lib" 2>/dev/null
  sleep 1
  if command -v lsof >/dev/null 2>&1 && lsof /dev/ttyUSB0 >/dev/null 2>&1; then
    echo "note: /dev/ttyUSB0 still busy - run: pkill -9 -f 'install/ars_base/lib'"
  else
    echo "stopped cleanly (serial port released)."
  fi
}
trap cleanup EXIT INT TERM

echo "waiting for the stack to come up..."
for _ in $(seq 1 45); do
  if ros2 topic list 2>/dev/null | grep -q '^/scan$'; then break; fi
  sleep 1
done

if ! ros2 topic list 2>/dev/null | grep -q '^/scan$'; then
  echo "WARNING: /scan never appeared. Check $LOG -- the lidar may not be connected."
  echo "The safety guard fails closed, so the robot will refuse to move without it."
fi

# Snappier hold-to-move: shorten the auto-repeat delay so a held key starts
# repeating quickly. Without this the first ~500 ms of every press stutters.
if command -v xset >/dev/null 2>&1 && [ -n "${DISPLAY:-}" ]; then
  xset r rate 200 40 2>/dev/null || true
fi

cat <<'EOF'

------------------------------------------------------------------
 HOLD-TO-MOVE teleop   (release the key and the robot stops)

   w / s   forward / backward
   a / d   strafe left / right   (mecanum, pure sideways)
   q / e   rotate left / right
   space   stop      + / -  speed      Ctrl-C  quit

 Obstacle guard is ACTIVE: motion toward anything closer than the
 stop distance is refused, but you can always reverse away.

 RViz shows the lidar map, the camera 3D cloud, and both camera feeds.

 Watch it work, in another terminal:
   ros2 topic echo /safety/blocked
   ros2 topic echo /obstacle_check/summary
------------------------------------------------------------------

EOF

ros2 run ars_base teleop_keys
