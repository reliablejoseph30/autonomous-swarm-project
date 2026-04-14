# Autonomous Swarm Project - Toba's Nodes
**MSc Advanced Drone Technology - University of the West of Scotland**
**Role: Telecommunications · Comms Resilience · Cyber Defence · Anti-Collision**

---

## Overview

This repository contains three ROS2 nodes built by Toba (Joseph) for the autonomous multi-drone swarm project. The nodes handle all communications, resilience, and cyber security aspects of the swarm.

The swarm consists of two drones operating in the Gazebo Classic Baylands simulation:
- **UAV0** - Iris multirotor, hovers at 30m AGL inspecting a red target box at (30, 20, 0)
- **UAV1** - Standard VTOL, orbits a 100m radius survey pattern at 100m AGL

---

## Toba's Three Nodes

### 1. relay_node.py ; Telecommunications Relay
Subscribes to both drone positions via MAVROS and rebroadcasts them on `/swarm/pose_array` so the entire swarm always knows where each drone is.

**What it does:**
- Subscribes to `/uav0/local_position/pose` and `/uav1/local_position/pose`
- Broadcasts both positions on `/swarm/pose_array` at 10Hz
- Monitors 3D distance between UAV0 and UAV1
- Fires `COMM_LOSS` alert when drones exceed 50m separation
- Publishes all alerts to `/net/alerts`
- Publishes health diagnostics to `/diagnostics`

---

### 2. comms_resilience_node.py ; Communications Resilience
Monitors link health, detects weather degradation, BVLOS occlusion, and anti-collision.

**What it does:**
- Measures message arrival rate (Hz) as a proxy for weather/signal degradation
- Classifies link condition: `CLEAR` (>90%) · `BREEZE` (70–90%) · `RAIN` (50–70%) · `STORM` (<50%)
- Heartbeat timeout — if no message arrives for 2 seconds after a healthy link, raises `OCCLUSION_ALERT` (drone behind obstacle; BVLOS)
- Anti-collision monitoring: `WARNING` at 10m, `DANGER` at 5m separation
- Publishes link health scores (0.0-1.0) per drone on `/comms/uav0/health` and `/comms/uav1/health`

---

### 3. cyber_defence_node.py; Cyber Defence
Defends against Craig's cyber attacks in real time.

**Attack detection:**

| Craig's Attack | Toba's Defence |
|---|---|
| Packet loss injection | Detects sudden message rate drop after healthy link |
| Communication jamming | Detects total silence after healthy link (3s timeout) |
| Position spoofing | Physics check — rejects positions requiring speed >30 m/s |
| Topic flooding (DoS) | Flags any topic arriving at >50 Hz |
| Fake position broadcasts | Cross-checks `/swarm/pose_array` against MAVROS ground truth |

**Additional:**
- HMAC signing framework; Craig cannot forge messages without the shared secret key
- Publishes threat level (`SAFE` / `WARNING` / `CRITICAL`) to `/cyber/status`
- Publishes attack-specific alerts to `/cyber/alerts`

---

## Live Operations Dashboard

A live web dashboard (`swarm_ops_dashboard.html`) that reads from ROS2 topics and displays:
- UAV0 and UAV1 live positions (x, y, z)
- Message rate per drone (Hz)
- 3D drone separation with collision warning
- Comms link health (CLEAR/BREEZE/RAIN/STORM)
- Cyber threat level (SAFE/WARNING/CRITICAL)
- Mission status
- Live alert feed from `/net/alerts`

---

## Topics

### Subscribed
| Topic | Type | Source |
|---|---|---|
| `/uav0/local_position/pose` | PoseStamped | MAVROS |
| `/uav1/local_position/pose` | PoseStamped | MAVROS |
| `/swarm/pose_array` | PoseArray | relay_node |

### Published
| Topic | Type | Description |
|---|---|---|
| `/swarm/pose_array` | PoseArray | Both drone positions |
| `/net/alerts` | String | Shared swarm alert bus |
| `/cyber/alerts` | String | Attack-specific alerts |
| `/cyber/status` | String | Current threat level |
| `/comms/uav0/health` | Float32 | UAV0 link quality (0-1) |
| `/comms/uav1/health` | Float32 | UAV1 link quality (0-1) |
| `/diagnostics` | DiagnosticArray | Node health status |

---

## Installation

### Copy nodes into the workspace
```bash
cp relay_node.py ~/ros2_ws/src/autonomous_swarm_project/autonomous_swarm_project/
cp comms_resilience_node.py ~/ros2_ws/src/autonomous_swarm_project/autonomous_swarm_project/
cp cyber_defence_node.py ~/ros2_ws/src/autonomous_swarm_project/autonomous_swarm_project/
cp setup.py ~/ros2_ws/src/autonomous_swarm_project/
```

### Rebuild workspace
```bash
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select autonomous_swarm_project --symlink-install
source install/setup.bash
```

---

## Running the Nodes

### Full launch sequence

**Terminal 1; Launch swarm:**
```bash
swarm
```

**Terminal 2; Fly the drones:**
```bash
cd ~/swarm_lab && python3 swarm_mission.py
```

**Terminal 3; relay_node:**
```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 run autonomous_swarm_project relay_node
```

**Terminal 4; comms_resilience_node:**
```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 run autonomous_swarm_project comms_resilience_node
```

**Terminal 5; cyber_defence_node:**
```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 run autonomous_swarm_project cyber_defence_node
```

**Terminal 6; Watch live alerts:**
```bash
source /opt/ros/humble/setup.bash
ros2 topic echo /net/alerts
```

**Terminal 7; Live dashboard bridge:**
```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
cd ~/swarm_lab
python3 swarm_dashboard_bridge.py
```
Then open `swarm_ops_dashboard.html` in your browser.

---

## Technical Stack

| Component | Detail |
|---|---|
| OS | Ubuntu 22.04 |
| ROS2 | Humble |
| Simulator | Gazebo Classic 11 |
| Flight stack | PX4 SITL v1.14 |
| Bridge | MAVROS |
| Language | Python 3 |
| QoS | BEST_EFFORT (required to match MAVROS publisher) |

> **Important:** MAVROS publishes with `BEST_EFFORT` reliability. All subscribers must use matching QoS or no messages will be received despite topics existing.

---

## Evidence of Working System

Confirmed working on 14 April 2026 with live drones flying in Gazebo Baylands world:

- relay_node receiving live positions at 30Hz
- COMM_LOSS alerts firing correctly at 89m+ separation
- comms_resilience_node classifying link as CLEAR at 100% health
- cyber_defence_node reporting THREAT_LEVEL=SAFE
- COLLISION_DANGER alert fired correctly when drones landed at same position (0.1m separation)
- OCCLUSION_ALERT fired correctly after mission ended and drones stopped transmitting
- Live dashboard displaying all metrics in real time

---

## GitHub Repository

**URL:** https://github.com/reliablejoseph30/autonomous-swarm-project

---

## Author

**Toba (Joseph)** — MSc Advanced Drone Technology
University of the West of Scotland
April 2026
