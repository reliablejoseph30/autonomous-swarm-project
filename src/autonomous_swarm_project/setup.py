from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'autonomous_swarm_project'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # This allows ROS 2 to find your YAML waypoints after installation
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='piotr',
    maintainer_email='piotr@todo.todo',
    description='Autonomous drone swarm search grid and 1500m mission execution.',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'mission_generator = autonomous_swarm_project.gen_waypoints:main',
            'mission_executor = autonomous_swarm_project.mission_executor:main',
            'image_logger = autonomous_swarm_project.image_logger:main',
	    'relay_node = autonomous_swarm_project.relay_node:main',
        ],
    },
)
