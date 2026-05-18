import os

from setuptools import find_packages, setup
from setuptools.command.develop import develop as _develop

package_name = 'flight_manager'


class develop(_develop):
    """`setup.py develop` that also writes a .pth file next to the egg-link.

    Background: setuptools >= 64 deprecated the legacy `develop` machinery and
    no longer creates `easy-install.pth`. Colcon's `--symlink-install` still
    invokes `setup.py develop`, so we end up with only an `*.egg-link` file in
    the install's `site-packages/` -- which setuptools knows how to follow at
    import time, but `importlib.metadata` does NOT. The ROS 2 console-script
    shim uses `importlib.metadata.distribution(...)` to look up entry points,
    so without a real `.pth` file pointing at the build directory (where the
    `*.egg-info` lives) every entry-point launched by ros2 dies with
    `PackageNotFoundError: No package metadata was found for flight-manager`.

    We restore the old behaviour locally: after the standard develop step we
    write a `<dist>.pth` file in the same site-packages directory, containing
    the absolute path to the package's build directory. That re-exposes the
    egg-info on sys.path and `importlib.metadata` finds it again.
    """

    def run(self):
        super().run()
        try:
            site_dir = self.install_dir
            if not site_dir:
                return
            egg_path = getattr(self, 'egg_path', None)
            if not egg_path:
                return
            target = os.path.abspath(os.path.join(site_dir, egg_path))
            pth_path = os.path.join(site_dir, f'{package_name}.pth')
            os.makedirs(site_dir, exist_ok=True)
            with open(pth_path, 'w', encoding='utf-8') as fh:
                fh.write(target + '\n')
        except Exception as exc:  # pragma: no cover - best effort
            import sys
            print(
                f'[{package_name}] warning: failed to write develop .pth file: {exc}',
                file=sys.stderr,
            )


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
    cmdclass={'develop': develop},
    entry_points={
        'console_scripts': [
            'mission_manager_node = flight_manager.mission_manager_node:main',
            'vision_fusion_node = flight_manager.vision_fusion_node:main',
            'single_source_router_node = flight_manager.single_source_router_node:main',
            'tf_broadcaster_node = flight_manager.tf_broadcaster_node:main',
            'gps_takeoff_land_node = flight_manager.gps_takeoff_land_node:main',
            'gps_waypoints_node = flight_manager.gps_waypoints_node:main',
            'gps_range_test_node = flight_manager.gps_range_test_node:main',
            'safety_log_supervisor_node = flight_manager.safety_log_supervisor_node:main',
        ],
    },
)
