import yaml
import os
import rclpy
from rclpy.node import Node

def generate_swarm_grid():
    """
    Generates an interleaved search grid for a 1500m mission.
    UAV1 and UAV2 maintain a 3.46m lateral offset to ensure full coverage.
    """
    ALT = 15.0
    LEN = 1500.0
    GAP = 3.46

    # Ensure the config directory exists so the build doesn't fail
    config_dir = 'config'
    if not os.path.exists(config_dir):
        os.makedirs(config_dir)

    # Coordinates for interleaved search
    mission = {
        "uav1": [
            {'x': 0.0, 'y': 0.0, 'z': -ALT},    # Lane 1 Start
            {'x': 0.0, 'y': LEN, 'z': -ALT},    # Lane 1 End
            {'x': GAP*2, 'y': LEN, 'z': -ALT},  # Lane 3 Start
            {'x': GAP*2, 'y': 0.0, 'z': -ALT}   # Lane 3 End
        ],
        "uav2": [
            {'x': GAP, 'y': LEN, 'z': -ALT},    # Lane 2 Start (North-to-South)
            {'x': GAP, 'y': 0.0, 'z': -ALT}     # Lane 2 End
        ]
    }

    file_path = os.path.join(config_dir, 'waypoints.yaml')
    with open(file_path, 'w') as f:
        yaml.dump(mission, f)
    
    print(f"SWARM SUCCESS: Interleaved 1500m Grid Generated at {file_path}")

def main(args=None):
    # Initialize the ROS 2 Python communication
    rclpy.init(args=args)
    
    # Run the grid generation logic
    generate_swarm_grid()
    
    # In a full mission, you would spin a node here. 
    # For a generator script, we simply shut down after completion.
    rclpy.shutdown()

if __name__ == "__main__":
    main()
