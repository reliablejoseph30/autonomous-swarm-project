Toba's Nodes - Swarm Project
Telecoms, Comms Resilience & Cyber Defence
==========================================

WHERE TO PLACE THE FILES:
Copy all 3 .py files into:
~/ros2_ws/src/autonomous_swarm_project/autonomous_swarm_project/

Replace the existing setup.py with the one provided at:
~/ros2_ws/src/autonomous_swarm_project/setup.py

Then rebuild:
  cd ~/ros2_ws
  source /opt/ros/humble/setup.bash
  colcon build --packages-select autonomous_swarm_project --symlink-install
  source install/setup.bash

HOW TO RUN TOBA'S NODES (after swarm is flying):
  source /opt/ros/humble/setup.bash
  source ~/ros2_ws/install/setup.bash
  ros2 run autonomous_swarm_project relay_node
  ros2 run autonomous_swarm_project comms_resilience_node
  ros2 run autonomous_swarm_project cyber_defence_node

WHAT EACH NODE DOES:

relay_node
  Subscribes to both drone positions and broadcasts them on /swarm/pose_array.
  Monitors communication range and separation between drones.
  Publishes alerts to /net/alerts.

comms_resilience_node
  Monitors heartbeat from both drones to detect occlusion and BVLOS signal loss.
  Measures message rate to simulate weather degradation (CLEAR/BREEZE/RAIN/STORM).
  Anti-collision monitoring with WARNING at 10m and DANGER at 5m.

cyber_defence_node
  Defends against Craig's attacks:
  - Packet loss injection detection
  - Communication jamming detection
  - Spoofed position detection (physics check)
  - Topic flooding / DoS detection
  - Fake position broadcast detection
  Publishes threat level to /cyber/status and alerts to /cyber/alerts.

TOPICS SUBSCRIBED TO:
  /uav0/local_position/pose
  /uav1/local_position/pose
  /swarm/pose_array

TOPICS PUBLISHED TO:
  /swarm/pose_array
  /net/alerts
  /cyber/alerts
  /cyber/status
  /comms/uav0/health
  /comms/uav1/health
  /diagnostics

Tested and confirmed working with live drones flying in
Gazebo baylands world on 4th April 2026.
