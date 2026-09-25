#!/usr/bin/env python3
"""Ensure a delayed cloud uses its paired odometry, not a newer pose."""
import os
from pathlib import Path
import signal
import subprocess
import time
import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header
from tf2_ros import StaticTransformBroadcaster

assert os.environ.get('ROS_DOMAIN_ID') == '173'
rclpy.init()
node = rclpy.create_node('odom_scan_timestamp_regression')
pub = node.create_publisher(Odometry, 'aft_mapped_to_init', 10)
cloud_pub = node.create_publisher(PointCloud2, 'cloud_registered', 10)
clouds = []
sub = node.create_subscription(PointCloud2, 'sensor_scan', clouds.append, 10)
broadcaster = StaticTransformBroadcaster(node)
t = TransformStamped()
t.header.frame_id = 'test_base'
t.child_frame_id = 'test_lidar'
t.transform.rotation.w = 1.0
broadcaster.sendTransform(t)
log = (Path(__file__).resolve().parents[1] / 'artifacts/localization_fix_20260924/odom_timestamp.log').open('w')
p = subprocess.Popen(['ros2', 'run', 'odom_bridge', 'odom_bridge_node', '--ros-args',
                      '-p', 'base_frame:=test_base', '-p', 'lidar_frame:=test_lidar',
                      '-p', 'robot_base_frame:=test_base'], stdout=log, stderr=subprocess.STDOUT,
                     start_new_session=True)

def spin(seconds):
    deadline = time.monotonic() + seconds
    while time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=.02)

def odom(stamp, x):
    m = Odometry(); m.header.stamp = stamp; m.header.frame_id = 'lidar_odom'
    m.pose.pose.orientation.w = 1.; m.pose.pose.position.x = x
    pub.publish(m)

try:
    deadline=time.monotonic()+8
    while pub.get_subscription_count()<2 and time.monotonic()<deadline:spin(.1)
    assert pub.get_subscription_count()>=2, 'Bridge not discovered'
    spin(.5)
    odom(node.get_clock().now().to_msg(), 0.)
    spin(.3)
    scan_stamp=node.get_clock().now().to_msg()
    odom(scan_stamp, 1.)
    spin(.2)
    odom(node.get_clock().now().to_msg(), 2.)  # newest pose is ahead of delayed scan
    spin(.2)
    cloud_pub.publish(point_cloud2.create_cloud_xyz32(
        Header(stamp=scan_stamp, frame_id='lidar_odom'), [[5.,0.,0.]]))
    spin(1.)
    assert clouds, 'No synchronized sensor scan'
    point=next(iter(point_cloud2.read_points(clouds[-1], field_names=('x','y','z'))))
    assert abs(float(point[0])-4.)<1e-5, f'Expected 5-1=4; got {point}'
    print('PASS delayed scan uses paired pose: x=4.0 (latest-pose bug would give 3.0)')
finally:
    if p.poll() is None:os.killpg(p.pid, signal.SIGINT)
    p.wait(timeout=10)
    log.close()
    node.destroy_node()
    rclpy.shutdown()
