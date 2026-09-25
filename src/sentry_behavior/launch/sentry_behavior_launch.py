# Copyright 2025 Lihan Chen
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.


import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace
from launch_ros.descriptions import ParameterFile
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    bringup_dir = get_package_share_directory("sentry_behavior")

    namespace = LaunchConfiguration("namespace")
    use_sim_time = LaunchConfiguration("use_sim_time")
    strategy = LaunchConfiguration("strategy")
    params_file = LaunchConfiguration("params_file")
    log_level = LaunchConfiguration("log_level")

    param_substitutions = {"use_sim_time": use_sim_time, "strategy": strategy}

    configured_params = ParameterFile(
        RewrittenYaml(
            source_file=params_file,
            root_key=namespace,
            param_rewrites=param_substitutions,
            convert_types=True,
        ),
        allow_substs=True,
    )

    stdout_linebuf_envvar = SetEnvironmentVariable(
        "RCUTILS_LOGGING_BUFFERED_STREAM", "1"
    )

    colorized_output_envvar = SetEnvironmentVariable("RCUTILS_COLORIZED_OUTPUT", "1")

    declare_namespace_cmd = DeclareLaunchArgument(
        "namespace",
        default_value="",
        description="Top-level namespace",
    )

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        "use_sim_time",
        default_value="false",
        description="Use simulation (Gazebo) clock if true",
    )

    declare_strategy_cmd = DeclareLaunchArgument(
        "strategy",
        default_value="rmuc_defend",
        description="State-machine strategy to run (rmuc_defend / a / b)",
    )

    declare_params_file_cmd = DeclareLaunchArgument(
        "params_file",
        default_value=os.path.join(bringup_dir, "params", "sentry_behavior.yaml"),
        description="Full path to the ROS2 parameters file to use for all launched nodes",
    )

    declare_log_level_cmd = DeclareLaunchArgument(
        "log_level", default_value="info", description="log level"
    )

    bringup_cmd_group = GroupAction(
        [
            PushRosNamespace(namespace=namespace),
            Node(
                package="sentry_behavior",
                executable="sentry_behavior_node",
                name="sentry_behavior_node",
                output="screen",
                parameters=[configured_params],
                arguments=["--ros-args", "--log-level", log_level],
            ),
        ]
    )

    ld = LaunchDescription()

    ld.add_action(stdout_linebuf_envvar)
    ld.add_action(colorized_output_envvar)

    ld.add_action(declare_namespace_cmd)
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_strategy_cmd)
    ld.add_action(declare_params_file_cmd)
    ld.add_action(declare_log_level_cmd)

    ld.add_action(bringup_cmd_group)

    return ld
