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
        ],
    },
)
