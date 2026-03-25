from setuptools import find_packages, setup

package_name = 'ardupilot_mavlink_bridge'

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
    maintainer='nvidia',
    maintainer_email='cvwiseworkk@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'mavlink_bridge_node = ardupilot_mavlink_bridge.mavlink_bridge_node:main',
            'telemetry_logger_node = ardupilot_mavlink_bridge.telemetry_logger_node:main'
        ],
    },
)
