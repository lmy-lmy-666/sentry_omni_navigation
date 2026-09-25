import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _launch_setup(context, *args, **kwargs):
    config = os.path.join(
        get_package_share_directory('rm_serial_driver'), 'config', 'serial_driver.yaml')

    device_override = LaunchConfiguration('device_name').perform(context).strip()
    params = [config]
    if device_override:
        params.append({'device_name': device_override})

    return [
        Node(
            package='rm_serial_driver',
            executable='rm_serial_driver_node',
            namespace='',
            output='screen',
            emulate_tty=True,
            parameters=params,
            arguments=['--ros-args', '--log-level', 'rm_serial_driver:=debug'],
        )
    ]


def generate_launch_description():
    device_name_arg = DeclareLaunchArgument(
        'device_name',
        default_value='',
        description='Override serial device (e.g. /dev/ttyACM0). Empty = use serial_driver.yaml.',
    )

    return LaunchDescription([
        device_name_arg,
        OpaqueFunction(function=_launch_setup),
    ])
