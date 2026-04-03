#!/bin/bash
echo "=========================================="
echo "  Autonomous Swarm Project - Launch Script"
echo "  Maintainer: Toba"
echo "=========================================="

# ── Step 1 — Kill any leftover processes ──────────────
echo "[1/7] Cleaning up old processes..."
pkill -9 -f mavros      2>/dev/null || true
pkill -9 -f gazebo_sitl 2>/dev/null || true
pkill -9 -f gzserver    2>/dev/null || true
pkill -9 -f gzclient    2>/dev/null || true
pkill -9 -x px4         2>/dev/null || true
sleep 3
echo "✅ Clean slate"

# ── Step 2 — Source ROS2 Foxy ─────────────────────────
echo "[2/7] Sourcing ROS2 Foxy..."
source /opt/ros/foxy/setup.bash

# ── Step 3 — Source Gazebo 11 ─────────────────────────
echo "[3/7] Sourcing Gazebo 11..."
source /usr/share/gazebo-11/setup.bash

# ── Step 4 — Source PX4 Gazebo ────────────────────────
echo "[4/7] Sourcing PX4 Gazebo..."
source /opt/PX4-Autopilot/Tools/setup_gazebo.bash \
  /opt/PX4-Autopilot \
  /opt/PX4-Autopilot/build/px4_sitl_default

# ── Step 5 — Clean old SDF files ─────────────────────
echo "[5/7] Cleaning old SDF files..."
rm -f /tmp/iris_0.sdf /tmp/iris_1.sdf
echo "✅ SDF files cleared"

# ── Step 6 — Launch Gazebo + PX4 ─────────────────────
echo "[6/7] Launching Gazebo + PX4 SITL..."
export DISPLAY=:0
cd /opt/PX4-Autopilot
./Tools/gazebo_sitl_multiple_run.sh -m iris -n 2 &
GAZEBO_PID=$!

echo "Waiting for PX4 to initialise (20 seconds)..."
sleep 20
echo "✅ PX4 should be ready"

# ── Step 7 — Start MAVROS ────────────────────────────
echo "[7/7] Starting MAVROS bridges..."

ros2 run mavros mavros_node --ros-args \
  -r __ns:=/uav0 \
  -p fcu_url:="udp://:14540@127.0.0.1:14557" \
  -p tgt_system:=1 \
  -p gcs_url:="" \
  > /tmp/mavros_uav0.log 2>&1 &
MAVROS0_PID=$!
echo "MAVROS UAV0 started (PID $MAVROS0_PID)"

sleep 3

ros2 run mavros mavros_node --ros-args \
  -r __ns:=/uav1 \
  -p fcu_url:="udp://:14541@127.0.0.1:14558" \
  -p tgt_system:=2 \
  -p gcs_url:="" \
  > /tmp/mavros_uav1.log 2>&1 &
MAVROS1_PID=$!
echo "MAVROS UAV1 started (PID $MAVROS1_PID)"

# ── Wait for heartbeat ────────────────────────────────
echo "Waiting for MAVROS heartbeat..."
sleep 5

# Check UAV0
for i in $(seq 1 30); do
  if ros2 topic list 2>/dev/null | grep -q "/uav0/mavros/state"; then
    echo "✅ UAV0 MAVROS heartbeat confirmed"
    break
  fi
  echo "  waiting for UAV0... ($i/30)"
  sleep 1
done

# Check UAV1
for i in $(seq 1 30); do
  if ros2 topic list 2>/dev/null | grep -q "/uav1/mavros/state"; then
    echo "✅ UAV1 MAVROS heartbeat confirmed"
    break
  fi
  echo "  waiting for UAV1... ($i/30)"
  sleep 1
done

# ── Ready ─────────────────────────────────────────────
echo ""
echo "=========================================="
echo "  ✅ SWARM READY"
echo "  UAV0 — iris drone (origin)"
echo "  UAV1 — iris drone (x=3)"
echo ""
echo "  Open a new terminal and run:"
echo "  docker exec -it swarm_dev bash"
echo "  source /opt/ros/foxy/setup.bash"
echo "  source /opt/autonomous_swarm_project/install/setup.bash"
echo ""
echo "  Then start Toba's nodes:"
echo "  ros2 run autonomous_swarm_project relay_node"
echo "  ros2 run autonomous_swarm_project comms_resilience_node"
echo "  ros2 run autonomous_swarm_project cyber_defence_node"
echo "=========================================="
echo ""
echo "Press Ctrl+C to stop everything"

# Keep script alive
wait $GAZEBO_PID
