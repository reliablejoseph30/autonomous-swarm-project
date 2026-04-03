<<<<<<< HEAD
# Autonomous Swarm Project – Team Development Environment

This container provides a fully reproducible development environment for the Autonomous Swarm Project. It includes:

- PX4 SITL (v1.13.x)
- ROS2 Foxy
- MAVSDK-Python
- Gazebo Classic
- Full swarm project source code
- All dependencies and environment configuration

Everything is pre-installed and ready to run.

------------------------------------------------------------
Directory Structure
------------------------------------------------------------

/opt
 ├── PX4-Autopilot              - PX4 SITL source
 ├── ros                        - ROS2 Foxy installation
 └── autonomous_swarm_project   - Swarm project workspace
      ├── src/                  - ROS2 packages
      ├── scripts/              - MAVSDK + mission scripts
      ├── sim/                  - Multi-UAV simulation scripts
      ├── config/               - YAML configs
      ├── build/                - Generated after colcon build
      ├── install/              - Generated after colcon build
      └── log/                  - Generated after colcon build

------------------------------------------------------------
Getting Started
------------------------------------------------------------

1. Source the workspace:

source /opt/autonomous_swarm_project/install/setup.bash

2. Build the workspace (if needed):

cd /opt/autonomous_swarm_project
colcon build

------------------------------------------------------------
Running PX4 SITL
------------------------------------------------------------

cd /opt/PX4-Autopilot
make px4_sitl gazebo

This launches PX4 SITL with Gazebo Classic.

------------------------------------------------------------
Running the Swarm (ROS2)
------------------------------------------------------------

cd /opt/autonomous_swarm_project
source install/setup.bash
ros2 launch <your_package> <your_launch_file>.launch.py

Replace <your_package> and <your_launch_file> with your actual swarm launch files.

------------------------------------------------------------
Running MAVSDK Scripts
------------------------------------------------------------

cd /opt/autonomous_swarm_project/scripts
python3 <script>.py

------------------------------------------------------------
Multi-UAV Simulation
------------------------------------------------------------

cd /opt/autonomous_swarm_project/sim
./start_swarm.sh

This script launches the multi-vehicle simulation environment.

------------------------------------------------------------
Notes
------------------------------------------------------------

- This environment is fully self-contained.
- No external dependencies are required.
- All teammates should use this container for consistent results.
- The container includes PX4, ROS2, MAVSDK, and the full swarm project.

------------------------------------------------------------
Maintainer
------------------------------------------------------------

Peter the Navigator
=======
# autonomous-swarm-project
MSc Advanced Drone Technology - Autonomous Multi-Drone Swarm Group Project
>>>>>>> 1fdee0aa065bb6d20f3bfc12b1ce22ac225646a9
