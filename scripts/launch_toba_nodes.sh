#!/bin/bash
echo "=========================================="
echo "  Toba's Nodes - Launch Script"
echo "  Relay | Comms Resilience | Cyber Defence"
echo "=========================================="

# Source ROS2 and workspace
source /opt/ros/foxy/setup.bash
source /opt/autonomous_swarm_project/install/setup.bash

echo "Starting relay_node..."
ros2 run autonomous_swarm_project relay_node &
sleep 1

echo "Starting comms_resilience_node..."
ros2 run autonomous_swarm_project comms_resilience_node &
sleep 1

echo "Starting cyber_defence_node..."
ros2 run autonomous_swarm_project cyber_defence_node &

echo ""
echo "=========================================="
echo "  ✅ All Toba's nodes running"
echo "  Monitoring /net/alerts for events"
echo "  Press Ctrl+C to stop all nodes"
echo "=========================================="

# Keep script alive
wait
