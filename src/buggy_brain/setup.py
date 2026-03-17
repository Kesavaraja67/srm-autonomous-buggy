from setuptools import find_packages, setup

package_name = 'buggy_brain'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='srmist',
    maintainer_email='srmist@todo.todo',
    description='SRM Autonomous Buggy Brain',
    license='TODO: License declaration',
    extras_require={
        'test': ['pytest'],
    },
    entry_points={
        'console_scripts': [
            'path_planner = buggy_brain.path_planner_node:main',
            'waypoint_follower = buggy_brain.waypoint_follower:main',
            'obstacle_detector = buggy_brain.obstacle_detector:main',
            'state_machine = buggy_brain.state_machine:main',
            'speed_controller = buggy_brain.speed_controller:main',
            'sensor_stub = buggy_brain.sensor_stub:main',
            'set_destination = buggy_brain.set_destination:main',
        ],
    },
)