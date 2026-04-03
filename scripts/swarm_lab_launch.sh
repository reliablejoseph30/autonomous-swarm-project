#!/usr/bin/env bash

set -e

PX4_DIR="${PX4_DIR:-$HOME/PX4-Autopilot}"
if [ ! -d "$PX4_DIR" ]; then
    echo "ERROR: PX4-Autopilot not found at $PX4_DIR"
    exit 1
fi

PX4_WORLDS="$PX4_DIR/Tools/simulation/gazebo-classic/sitl_gazebo-classic/worlds"
PX4_MODELS="$PX4_DIR/Tools/simulation/gazebo-classic/sitl_gazebo-classic/models"
PX4_SCRIPTS="$PX4_DIR/Tools/simulation/gazebo-classic/sitl_gazebo-classic/scripts"
BUILD_PATH="$PX4_DIR/build/px4_sitl_default"
AUGMENTED_WORLD="/tmp/baylands_swarm_augmented.world"

if [ ! -f "$PX4_WORLDS/baylands.world" ]; then
    echo "ERROR: baylands.world not found"
    echo "Run: cd ~/PX4-Autopilot && make px4_sitl_default gazebo-classic"
    exit 1
fi

# Clean up old runs
pkill -x px4 2>/dev/null || true
pkill -f gzserver 2>/dev/null || true
pkill -f gzclient 2>/dev/null || true
pkill -f mavros 2>/dev/null || true
sleep 2

source /opt/ros/humble/setup.bash
export ROS_VERSION=2
cd "$PX4_DIR"
source Tools/simulation/gazebo-classic/setup_gazebo.bash \
    "$(pwd)" "$(pwd)/build/px4_sitl_default"

# ── Augment the baylands world ───────────────────────────────────────────────
python3 - "$PX4_WORLDS/baylands.world" "$AUGMENTED_WORLD" << 'PYEOF'
import sys
import xml.etree.ElementTree as ET

ET.register_namespace('', '')
src_path, dst_path = sys.argv[1], sys.argv[2]

tree = ET.parse(src_path)
root = tree.getroot()
world = root.find('world')

target_xml = '''<model name="inspection_target">
  <static>true</static>
  <pose>30 20 0 0 0 0</pose>
  <link name="link">
    <collision name="collision">
      <geometry><box><size>8 6 4</size></box></geometry>
    </collision>
    <visual name="visual">
      <geometry><box><size>8 6 4</size></box></geometry>
      <material>
        <script>
          <uri>file://media/materials/scripts/gazebo.material</uri>
          <name>Gazebo/Red</name>
        </script>
      </material>
    </visual>
  </link>
</model>'''

for xml_str in [target_xml]:
    world.append(ET.fromstring(xml_str))

tree.write(dst_path, xml_declaration=True, encoding='unicode')
print(f"[setup] Augmented world written to {dst_path}")
PYEOF

cleanup() {
    echo "[shutdown] Stopping all processes..."
    kill ${MAVROS1_PID:-} ${MAVROS0_PID:-} 2>/dev/null || true
    pkill -x px4 2>/dev/null || true
    pkill -f gzserver 2>/dev/null || true
    pkill -f gzclient 2>/dev/null || true
    pkill -f mavros 2>/dev/null || true
    echo "[shutdown] Done."
}
trap cleanup SIGINT SIGTERM

echo ""
echo "╔══════════════════════════════════════════════════════╗"
echo "║  Swarm Lab — Baylands  (Gazebo Classic + PX4 SITL)  ║"
echo "╚══════════════════════════════════════════════════════╝"
echo ""

# ── [1/4] Single gzserver with augmented world ───────────────────────────────
echo "[1/4] Starting Gazebo Classic..."
gzserver "$AUGMENTED_WORLD" --verbose \
    -s libgazebo_ros_init.so \
    -s libgazebo_ros_factory.so &
GZSERVER_PID=$!
sleep 8

gzclient &
GZCLIENT_PID=$!

# ── [2/4] UAV0: Iris, instance 1, TCP 4561 ───────────────────────────────────
# sitl_multiple_run.sh uses jinja to generate SDFs and starts instances at N=1.
# We replicate spawn_model() directly so we control the world file.
echo "[2/4] Spawning UAV0: Iris (instance 1)..."

UAV0_DIR="$BUILD_PATH/rootfs/1"
mkdir -p "$UAV0_DIR"
pushd "$UAV0_DIR" > /dev/null

export PX4_SIM_MODEL=gazebo-classic_iris
"$BUILD_PATH/bin/px4" -i 1 -d "$BUILD_PATH/etc" > /tmp/px4_uav0.log 2>&1 &
UAV0_PID=$!

python3 "$PX4_SCRIPTS/jinja_gen.py" \
    "$PX4_MODELS/iris/iris.sdf.jinja" \
    "$PX4_DIR/Tools/simulation/gazebo-classic/sitl_gazebo-classic" \
    --mavlink_tcp_port 4561 \
    --mavlink_udp_port 14561 \
    --mavlink_id 1 \
    --gst_udp_port 5601 \
    --video_uri 5601 \
    --mavlink_cam_udp_port 14531 \
    --output-file /tmp/iris_1.sdf

gz model --spawn-file=/tmp/iris_1.sdf --model-name=iris_1 -x 0 -y 0 -z 0.83
popd > /dev/null

WAIT_COUNT=0
until grep -q "Ready for takeoff" /tmp/px4_uav0.log 2>/dev/null; do
    sleep 2; WAIT_COUNT=$((WAIT_COUNT + 2))
    echo "    UAV0 waiting (${WAIT_COUNT}s)..."
    if [ $WAIT_COUNT -gt 120 ]; then
        echo "ERROR: UAV0 did not connect. Log:"; tail -40 /tmp/px4_uav0.log; exit 1
    fi
done
echo "[2/4] UAV0 connected."
sleep 3

# ── [3/4] UAV1: Standard VTOL, instance 2, TCP 4562 ─────────────────────────
echo "[3/4] Spawning UAV1: Standard VTOL (instance 2)..."

UAV1_DIR="$BUILD_PATH/rootfs/2"
mkdir -p "$UAV1_DIR"
pushd "$UAV1_DIR" > /dev/null

export PX4_SIM_MODEL=gazebo-classic_standard_vtol
"$BUILD_PATH/bin/px4" -i 2 -d "$BUILD_PATH/etc" > /tmp/px4_uav1.log 2>&1 &
UAV1_PID=$!

python3 "$PX4_SCRIPTS/jinja_gen.py" \
    "$PX4_MODELS/standard_vtol/standard_vtol.sdf.jinja" \
    "$PX4_DIR/Tools/simulation/gazebo-classic/sitl_gazebo-classic" \
    --mavlink_tcp_port 4562 \
    --mavlink_udp_port 14562 \
    --mavlink_id 2 \
    --gst_udp_port 5602 \
    --video_uri 5602 \
    --mavlink_cam_udp_port 14532 \
    --output-file /tmp/standard_vtol_2.sdf

gz model --spawn-file=/tmp/standard_vtol_2.sdf --model-name=standard_vtol_2 -x 20 -y 0 -z 0.83
popd > /dev/null

WAIT_COUNT=0
until grep -q "Ready for takeoff" /tmp/px4_uav1.log 2>/dev/null; do
    sleep 2; WAIT_COUNT=$((WAIT_COUNT + 2))
    echo "    UAV1 waiting (${WAIT_COUNT}s)..."
    if [ $WAIT_COUNT -gt 120 ]; then
        echo "ERROR: UAV1 did not connect. Log:"; tail -40 /tmp/px4_uav1.log; exit 1
    fi
done
echo "[3/4] UAV1 connected."
sleep 3

# ── [4/4] MAVROS ─────────────────────────────────────────────────────────────
# Instance N uses MAVLink UDP: PX4 listens on 14580+N, MAVROS listens on 14540+N
echo "[4/4] Starting MAVROS..."
ros2 launch mavros px4.launch \
    fcu_url:="udp://:14541@localhost:14581" \
    gcs_url:="udp://@localhost:14550" \
    tgt_system:=2 \
    tgt_component:=1 \
    namespace:=/uav0 &
MAVROS0_PID=$!

sleep 4

ros2 launch mavros px4.launch \
    fcu_url:="udp://:14542@localhost:14582" \
    gcs_url:="udp://@localhost:14550" \
    tgt_system:=3 \
    tgt_component:=1 \
    namespace:=/uav1 &
MAVROS1_PID=$!

sleep 5
echo ""
echo "════════════════════════════════════════════════════════"
echo "  All processes running."
echo ""
echo "  UAV0 (iris)         instance 1 | TCP 4561 | UDP 14561"
echo "  UAV1 (standard_vtol) instance 2 | TCP 4562 | UDP 14562"
echo ""
echo "  MAVROS:  /uav0/state   /uav1/state"
echo ""
echo "  Terminal 2: source /opt/ros/humble/setup.bash"
echo "              python3 swarm_mission.py"
echo ""
echo "  Ctrl+C to shut down."
echo "════════════════════════════════════════════════════════"

wait
