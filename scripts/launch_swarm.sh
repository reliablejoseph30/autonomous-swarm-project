#!/bin/bash

echo "=========================================="
echo "  Autonomous Swarm Project - Launch Script"
echo "  Maintainer: Toba"
echo "=========================================="

# Step 1 - Source ROS2 Foxy

echo "[1/4] Sourcing ROS2 Foxy..."
source /opt/ros/foxy/setup.bash

# Step 2 - Source Gazebo 11

echo "[2/4] Sourcing Gazebo 11..."
source /usr/share/gazebo-11/setup.bash

# Step 3 - Source PX4 Gazebo setup

echo "[3/4] Sourcing PX4 Gazebo..."
source /opt/PX4-Autopilot/Tools/setup_gazebo.bash \
  /opt/PX4-Autopilot \
  /opt/PX4-Autopilot/build/px4_sitl_default

# Step 4 - Clean old SDF files

echo "[4/4] Cleaning old SDF files..."
rm -f /tmp/iris_0.sdf /tmp/iris_1.sdf

# Launch simulation

echo ""
echo "Launching 2-drone swarm in Gazebo..."
echo "Press Ctrl+C to stop"
echo ""
cd /opt/PX4-Autopilot
./Tools/gazebo_sitl_multiple_run.sh -m iris -n 2
