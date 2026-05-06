"""Standalone launcher for the ORB-SLAM3 bridge.

Useful when you want to test ORB-SLAM3 in isolation (without the full
flight stack). The :mod:`bringup` flight launch wires the same node into
the system based on ``navigation`` / ``vo_backend`` parameters.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    pkg_share = get_package_share_directory("orb_slam3_bridge")

    default_settings = os.path.join(
        pkg_share, "config", "orb_slam3_monocular_template.yaml")
    default_vocab = os.path.join(
        os.environ.get("ORB_SLAM3_ROOT", "/opt/ORB_SLAM3"),
        "Vocabulary",
        "ORBvoc.txt",
    )

    sensor_arg = DeclareLaunchArgument(
        "sensor_type", default_value="MONOCULAR",
        description="MONOCULAR or IMU_MONOCULAR")
    vocab_arg = DeclareLaunchArgument(
        "vocabulary_path", default_value=default_vocab)
    settings_arg = DeclareLaunchArgument(
        "settings_path", default_value=default_settings)
    image_arg = DeclareLaunchArgument(
        "image_topic", default_value="/camera/image_raw")
    imu_arg = DeclareLaunchArgument(
        "imu_topic", default_value="/ap/imu/raw")
    pose_arg = DeclareLaunchArgument(
        "pose_topic", default_value="/vo/pose")
    nav_status_arg = DeclareLaunchArgument(
        "nav_status_topic", default_value="/vo/nav_status")
    viewer_arg = DeclareLaunchArgument(
        "enable_viewer", default_value="false")

    node = Node(
        package="orb_slam3_bridge",
        executable="orb_slam3_node",
        name="orb_slam3_node",
        output="screen",
        parameters=[{
            "vocabulary_path": LaunchConfiguration("vocabulary_path"),
            "settings_path": LaunchConfiguration("settings_path"),
            "sensor_type": LaunchConfiguration("sensor_type"),
            "image_topic": LaunchConfiguration("image_topic"),
            "imu_topic": LaunchConfiguration("imu_topic"),
            "pose_topic": LaunchConfiguration("pose_topic"),
            "nav_status_topic": LaunchConfiguration("nav_status_topic"),
            "enable_viewer": LaunchConfiguration("enable_viewer"),
        }],
    )

    return LaunchDescription([
        sensor_arg,
        vocab_arg,
        settings_arg,
        image_arg,
        imu_arg,
        pose_arg,
        nav_status_arg,
        viewer_arg,
        node,
    ])
