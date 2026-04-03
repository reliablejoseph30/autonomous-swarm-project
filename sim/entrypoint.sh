#!/bin/bash

# 1. Launch Multi-Drone Gazebo World (2 UAVs)
cd ~/PX4-Autopilot
./Tools/gazebo_sitl_multiple_run.sh -m iris -n 2 &
sleep 180   # 3-minute safety stagger

# 2. Launch Logging (The "Spine")
python3 ~/autonomous_swarm_project/scripts/image_logger.py &

# 3. Global Simulation Timer (25 minutes)
sleep 1500
echo "SWARM MISSION COMPLETE: Terminating all nodes."
pkill -f px4
