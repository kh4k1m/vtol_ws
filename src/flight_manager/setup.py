from setuptools import find_packages, setup

package_name = 'flight_manager'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='nvidia',
    maintainer_email='cvwiseworkk@gmail.com',
    description='Mission-level flight manager for autonomous takeoff and landing.',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'mission_manager_node = flight_manager.mission_manager_node:main',
            'vision_fusion_node = flight_manager.vision_fusion_node:main',
            'single_source_router_node = flight_manager.single_source_router_node:main',
            'tf_broadcaster_node = flight_manager.tf_broadcaster_node:main',
            'gps_takeoff_land_node = flight_manager.gps_takeoff_land_node:main',
            'gps_waypoints_node = flight_manager.gps_waypoints_node:main',
            'gps_range_test_node = flight_manager.gps_range_test_node:main',
        ],
    },
)
