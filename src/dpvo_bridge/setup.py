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
        ('share/' + package_name, ['README.md']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='OpenAI',
    maintainer_email='user@example.com',
    description='ROS 2 wrapper around DPVO plus CSV logger for GPS and predicted trajectory.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'dpvo_node = dpvo_bridge.dpvo_node:main',
            'trajectory_logger_node = dpvo_bridge.trajectory_logger_node:main',
        ],
    },
)
