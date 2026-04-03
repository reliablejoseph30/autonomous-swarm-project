# Autonomous Swarm Project — Progress Log

---

## Session 1 — 23 March 2026
**Lead:** Toba (Telecommunications & Relay Infrastructure)

---

### Team Members & Roles

| Name   | Role                                      |
|--------|-------------------------------------------|
| Peter  | Navigation & Collision Avoidance          |
| Connor | Object Detection & Perception             |
| Toba   | Telecommunications & Relay Infrastructure |
| Craig  | Cyber Security & Network Resilience       |

---

### Environment Setup & Fixes

#### 1. Docker Image Loaded
- Downloaded `swarm_dev_team_ready.tar` from SharePoint
- Loaded into Docker using `docker load -i ~/swarm_dev_team_ready.tar`
- Container named `swarm_dev` running `swarm_dev_team_ready:latest`

#### 2. Docker Permissions Fixed
```bash
sudo usermod -aG docker $USER
sudo reboot
```

#### 3. Docker Run Command (Full — use this every time from scratch)
```bash
xhost +local:docker

docker run -it \
  --name swarm_dev \
  --privileged \
  -e DISPLAY=$DISPLAY \
  -e LIBGL_ALWAYS_SOFTWARE=1 \
  -e GAZEBO_IP=127.0.0.1 \
  -e LIBGL_DRI3_DISABLE=1 \
  -e MESA_GL_VERSION_OVERRIDE=3.3 \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  --device /dev/dri \
  swarm_dev_team_ready:latest \
  bash
```

#### 4. Daily Start Command (container already exists)
```bash
xhost +local:docker
docker start swarm_dev
docker exec -it swarm_dev bash
```

#### 5. Missing ROS2 Gazebo Plugins Fixed
```bash
apt-get install -y ros-foxy-gazebo-ros-pkgs
```

#### 6. Gazebo Shader Path Fixed
```bash
source /usr/share/gazebo-11/setup.bash
```

#### 7. SDF Overwrite Protection Fix
```bash
rm -f /tmp/iris_0.sdf /tmp/iris_1.sdf
```

#### 8. Waypoints Path Fix
```bash
ln -s /opt/autonomous_swarm_project /root/autonomous_swarm_project
```

#### 9. px4_msgs Built From Source
```bash
cd /opt
git clone https://github.com/PX4/px4_msgs.git
cd px4_msgs
git checkout release/1.13
mv /opt/px4_msgs /opt/autonomous_swarm_project/src/
cd /opt/autonomous_swarm_project
colcon build --packages-select px4_msgs --symlink-install
source install/setup.bash
```

---

### Launch Sequence (Use Every Time)

#### Step 1 — On laptop terminal:
```bash
xhost +local:docker
docker start swarm_dev
```

#### Step 2 — Open VSCode, attach to container:
- Ctrl+Shift+P → "Attach to Running Container" → swarm_dev
- Or on terminal: `docker exec -it swarm_dev bash`

#### Step 3 — Terminal 1 (Gazebo):
```bash
source /opt/ros/foxy/setup.bash
source /usr/share/gazebo-11/setup.bash
source /opt/PX4-Autopilot/Tools/setup_gazebo.bash \
  /opt/PX4-Autopilot \
  /opt/PX4-Autopilot/build/px4_sitl_default
rm -f /tmp/iris_0.sdf /tmp/iris_1.sdf
cd /opt/PX4-Autopilot
./Tools/gazebo_sitl_multiple_run.sh -m iris -n 2
```

#### Step 4 — Terminal 2 (Mission Executor):
```bash
source /opt/ros/foxy/setup.bash
source /opt/autonomous_swarm_project/install/setup.bash
ros2 run autonomous_swarm_project mission_executor
```

#### Step 5 — Terminal 3 (Relay Node — Toba):
```bash
source /opt/ros/foxy/setup.bash
source /opt/autonomous_swarm_project/install/setup.bash
ros2 run autonomous_swarm_project relay_node
```

---

### Toba's Contribution — Relay Node

**File:** src/autonomous_swarm_project/autonomous_swarm_project/relay_node.py

**What it does:**
- Subscribes to both drone positions
- Broadcasts positions on /swarm/pose_array so drones know each other's location
- Monitors distance between UAV1 and UAV2
- Publishes communication alerts to /net/alerts when signal degrades
- Publishes health diagnostics to /diagnostics

**Parameters aligned with Peter's specs:**
- Corridor width: 3.47m
- Minimum safe gap: 2.0m
- Soft geofence buffer: 2.0m
- Hard geofence: 0.5m
- Simulated comm range limit: 50.0m

**Topics published:**
- /swarm/pose_array — both drone positions broadcast to swarm
- /net/alerts — communication warnings and alerts
- /diagnostics — health status of swarm communications

**Topics subscribed (current — raw PX4):**
- /px4_1/fmu/out/vehicle_local_position
- /px4_2/fmu/out/vehicle_local_position

**Topics subscribed (next step — MAVROS):**
- /uav1/mavros/local_position/pose
- /uav2/mavros/local_position/pose

---

### MAVROS — Next Priority

Peter confirmed MAVROS is the correct bridge between ROS2 and PX4.
px4_ros_com is NOT needed. MAVROS is already installed in the container at:
- /opt/ros/foxy/lib/mavros
- /opt/ros/foxy/share/mavros

Once Peter sets up the MAVROS launch, drone positions will publish on:
- /uav1/mavros/local_position/pose
- /uav2/mavros/local_position/pose

relay_node.py will be updated to subscribe to these MAVROS topics.

---

### Known Issues & Blockers

| Issue | Status |
|-------|--------|
| Drones not flying — MAVROS bridge not yet configured | Blocked — Peter to fix |
| start_swarm.sh not yet created | Pending Peter |
| relay_node.py topics need updating once MAVROS is live | Pending Peter's MAVROS setup |

---

### Next Steps Per Team Member

#### Peter (Navigation)
- Configure MAVROS to bridge ROS2 and PX4
- Create start_swarm.sh
- Get both drones flying via MAVROS offboard control

#### Toba (Telecoms & Relay)
- Update relay_node.py to use MAVROS position topics
- Test relay node with live drone positions
- Begin signal strength simulation between drones

#### Connor (Perception)
- Integrate camera feed from Gazebo
- Begin YOLO integration on simulated imagery

#### Craig (Cyber Security)
- Begin IDS node planning
- Design fault injection test strategy

---

### Files Shared on SharePoint Today

| File | Description |
|------|-------------|
| relay_node.py | Toba's telecoms relay node |
| launch_swarm.sh | Launch script for simulation |
| PROGRESS_LOG.md | This document |

---

*Maintained by Toba — Telecommunications & Relay Infrastructure*
*Last updated: 23 March 2026*