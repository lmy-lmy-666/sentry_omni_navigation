import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _resolve_profile(profile_arg, pkg_share):
    """A bare filename (rmuc_blue.yaml) → config/profiles/<name>; any path with
    a separator is used as-is. Lets you pass either form on the command line."""
    # profile_arg may be a string (from default) or a list-of-fragments substitution;
    # collapse to a string for the bare-name check.
    if isinstance(profile_arg, list):
        s = "".join(str(x) for x in profile_arg)
    else:
        s = str(profile_arg)
    if s and os.sep not in s and "/" not in s:
        return os.path.join(pkg_share, "config", "profiles", s)
    return s


def _make_node(context, *, pkg_share, default_profile):
    raw = LaunchConfiguration("profile", default=default_profile).perform(context)
    profile = _resolve_profile(raw, pkg_share)
    return [
        Node(
            package="omni_decision_sample",
            executable="omni_decision_sample_node",
            name="omni_decision_sample",
            output="screen",
            parameters=[{
                # 战术配置文件（YAML）：巡逻点、补给点、开局站位、起伏段等
                "profile_path": profile,

                # FSM 主循环频率 (Hz)，每隔 1/tick_frequency 秒评估一次状态转移
                "tick_frequency": 10.0,

                # Nav2 不可用时的 fallback 方案 —— 通过 PoseStamped topic 发送目标点，
                # 代替 Nav2 action。接收方自己判断是否到达（odom + TF）
                "goal_topic": "/goal_pose",

                # Nav2 导航 action 名称，默认 navigate_to_pose
                # 哨兵一般只有一个 Nav2 planner，用默认值即可
                "nav_action_name": "navigate_to_pose",

                # 目标点的参考坐标系，通常是 map 或 odom
                "goal_frame": "map",

                # 判定到达目标点的距离容差 (m)
                # 哨兵底盘较大、定位噪声高，0.25m 比默认的 0.1m 更不容易反复摆动
                "goal_reached_distance_tolerance": 0.25,
            }],
        ),
    ]


def generate_launch_description():
    pkg_share = get_package_share_directory("omni_decision_sample")
    default_profile = os.path.join(pkg_share, "config", "profiles", "rmuc_red.yaml")

    return LaunchDescription([
        DeclareLaunchArgument(
            "profile",
            default_value=default_profile,
            description=(
                "Tactic profile. Bare filename (e.g. rmuc_blue.yaml) is resolved "
                "against config/profiles/; an absolute/relative path is used as-is."
            ),
        ),
        OpaqueFunction(
            function=lambda context: _make_node(
                context, pkg_share=pkg_share, default_profile=default_profile)
        ),
    ])
