from setuptools import find_packages, setup

package_name = 'dpvo_bridge'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/dpvo_with_logger.launch.py']),
        ('share/' + package_name + '/config', ['config/dpvo_params.yaml']),
        ('share/' + package_name + '/scripts', [
            'scripts/run_dpvo_node.sh',
            'scripts/run_dpvo_node_jetson.sh',
        ]),
        ('share/' + package_name, ['README.md']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='vtol',
    maintainer_email='vtol@example.com',
    description=(
        'ROS 2 wrapper around DPVO publishing to the vision_interfaces topic '
        'contract. Includes a trajectory writer that produces TUM, CSV and a '
        'PNG flyover map.'
    ),
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'dpvo_node = dpvo_bridge.dpvo_node:main',
            # New writer for the DPVO-only mission launch (TUM + CSV + PNG map).
            'dpvo_trajectory_writer_node = '
            'dpvo_bridge.dpvo_trajectory_writer_node:main',
            # Kept for backwards compatibility with the existing
            # flight.launch.py (DPVO-vs-GPS CSV only).
            'trajectory_logger_node = dpvo_bridge.trajectory_logger_node:main',
        ],
    },
)
