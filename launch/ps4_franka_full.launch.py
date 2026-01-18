from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    fake = LaunchConfiguration("fake")
    use_rviz = LaunchConfiguration("rviz")

    franka_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("mul_franka_launch"),
                "launch",
                "franka_moveit_camera.launch.py",
            )
        ),
        launch_arguments={"fake": fake}.items(),
    )

    rviz_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("mul_franka_launch"),
                "launch",
                "rviz.launch.py",
            )
        ),
        condition=IfCondition(use_rviz),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "fake",
                default_value="true",
                description="Use fake hardware for Franka (true/false).",
            ),
            DeclareLaunchArgument(
                "rviz",
                default_value="true",
                description="Launch RViz (true/false).",
            ),
            franka_launch,
            Node(
                package="joy",
                executable="joy_node",
                name="joy_node",
                output="screen",
            ),
            rviz_launch,
            Node(
                package="ps4_franka_servo",
                executable="ps4_franka_servo",
                name="ps4_franka_servo",
                output="screen",
            ),
        ]
    )
