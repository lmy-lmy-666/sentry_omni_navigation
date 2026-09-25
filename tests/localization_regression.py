#!/usr/bin/env python3
"""Isolated ROS integration checks. Run with the workspace sourced and ROS_DOMAIN_ID=173."""
import math
import os
from pathlib import Path
import signal
import subprocess
import tempfile
import time

import numpy as np
import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped, TransformStamped
from nav_msgs.msg import OccupancyGrid
from rclpy.qos import QoSProfile, DurabilityPolicy
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header, Bool
from tf2_msgs.msg import TFMessage
from tf2_ros import StaticTransformBroadcaster

ROOT = Path(__file__).resolve().parents[1]
OUT = ROOT / 'artifacts/localization_fix_20260924'
OUT.mkdir(exist_ok=True, parents=True)
assert os.environ.get('ROS_DOMAIN_ID') == '173', 'Use isolated ROS_DOMAIN_ID=173'
rclpy.init()
node = rclpy.create_node('localization_regression')
processes = []
handles = []

def start(args, name):
    f = (OUT / name).open('w')
    handles.append(f)
    p = subprocess.Popen(args, stdout=f, stderr=subprocess.STDOUT, start_new_session=True)
    processes.append(p)
    return p

def stop():
    for p in reversed(processes):
        if p.poll() is None:
            os.killpg(p.pid, signal.SIGINT)
            try:
                p.wait(timeout=15)
            except subprocess.TimeoutExpired:
                os.killpg(p.pid, signal.SIGKILL)
                p.wait()
    processes.clear()

def wait(predicate, timeout, tick=None):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if tick:
            tick()
        rclpy.spin_once(node, timeout_sec=0.04)
        if predicate():
            return True
    return False

try:
    maps = []
    qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
    sub = node.create_subscription(OccupancyGrid, '/map', lambda m: maps.append(m), qos)
    for composed in ('True', 'False'):
        maps.clear()
        if composed == 'True':
            start(['ros2', 'run', 'rclcpp_components', 'component_container_mt', '--ros-args', '-r', '__node:=nav2_container'], 'container.log')
        t = time.monotonic()
        start(['ros2', 'launch', 'sentry_nav_bringup', 'localization_launch.py',
               f'map:={ROOT}/src/sentry_nav_bringup/map/reality/0927.yaml',
               f'prior_pcd_file:={ROOT}/src/sentry_nav_bringup/pcd/reality/0927.pcd',
               f'params_file:={ROOT}/src/sentry_nav_bringup/config/reality/nav2_params.yaml',
               f'use_composition:={composed}'], f'map_startup_{composed}.log')
        assert wait(lambda: bool(maps), 15), 'Map did not arrive independently of PCD preprocessing'
        print(f'PASS real 0927 map, composition={composed}: {time.monotonic()-t:.3f}s', flush=True)
        assert wait(lambda: 'Manual override:' in (OUT / f'map_startup_{composed}.log').read_text(), 30), 'PCD preprocessing did not complete'
        wait(lambda: False, .5)
        stop()
        # Drain DDS callbacks from the stopped publishers before next case.
        wait(lambda: False, 1)
    node.destroy_subscription(sub)

    with tempfile.TemporaryDirectory() as tmp:
        rng = np.random.default_rng(19)
        # Asymmetric vertical surfaces, giving a unique nearby planar solution.
        pts = []
        for x, y, sx, sy in [(2, 1, 2, .05), (-2, 3, .04, 1.5), (1,-2,1,.04), (-3,-1,.03,.7)]:
            for _ in range(600):
                pts.append([x+rng.uniform(-sx,sx), y+rng.uniform(-sy,sy), rng.uniform(.1,1.5)])
        # Dense horizontal surfaces must not become XY obstacles. Source and
        # target deliberately have different floor/ceiling coverage.
        walls = np.asarray(pts, dtype=np.float32)
        horizontal = np.c_[rng.uniform(-6, 6, (6000, 2)), rng.choice([0., 3.], 6000)]
        target = np.vstack([walls, horizontal]).astype(np.float32)
        pts = target.tolist()
        pcd = Path(tmp)/'test.pcd'
        pcd.write_text('VERSION .7\nFIELDS x y z\nSIZE 4 4 4\nTYPE F F F\nCOUNT 1 1 1\nWIDTH %d\nHEIGHT 1\nPOINTS %d\nDATA ascii\n' % (len(pts),len(pts)) + '\n'.join(' '.join(map(str,p)) for p in target))
        yaw=.08
        rot=np.array([[math.cos(yaw),-math.sin(yaw),0],[math.sin(yaw),math.cos(yaw),0],[0,0,1]])
        source=(walls-np.array([.4,-.3,0]))@rot
        clutter = np.c_[rng.uniform(1, 8, (10000, 2)), rng.choice([0., 3.], 10000)]
        source=np.vstack([source, clutter])
        pose=[]
        validity=[]
        valid_sub=node.create_subscription(Bool, "localization_valid", lambda m: validity.append(m.data), qos)
        def tf_callback(msg):
            for t in msg.transforms:
                if t.header.frame_id=='map' and t.child_frame_id=='odom':
                    pose[:] = [t.transform.translation.x,t.transform.translation.y,2*math.atan2(t.transform.rotation.z,t.transform.rotation.w)]
        tf_sub=node.create_subscription(TFMessage,'/tf',tf_callback,20)
        broadcaster=StaticTransformBroadcaster(node)
        t=TransformStamped();t.header.stamp=node.get_clock().now().to_msg();t.header.frame_id='odom';t.child_frame_id='base_footprint';t.transform.rotation.w=1.
        broadcaster.sendTransform(t)
        pub=node.create_publisher(point_cloud2.PointCloud2,'registered_scan',10)
        initial=node.create_publisher(PoseWithCovarianceStamped,'initialpose',10)
        args=['ros2','run','small_gicp_relocalization','small_gicp_relocalization_node','--ros-args']
        params={'registration_height_filter':'true','prior_pcd_file':str(pcd),'base_frame':'base_footprint','accumulated_count_threshold':6,'global_leaf_size':.15,'registered_leaf_size':.15,'max_iterations':50,'enable_periodic_relocalization':'true','relocalization_interval':.5,'max_fitness_error':.3,'min_range':0.0}
        for k,v in params.items():args+=['-p',f'{k}:={v}']
        start(args,'synthetic.log')
        assert wait(lambda: pub.get_subscription_count() > 0, 8), 'Localizer not discovered'
        wait(lambda: False, .2)
        assert wait(lambda: len(pose)==3 and bool(validity), 3), 'Startup TF/status missing'
        assert not validity[-1], 'Bootstrap TF must not be marked localized'
        print('PASS bootstrap map->odom available with localization_valid=false', flush=True)
        def publish():
            pub.publish(point_cloud2.create_cloud_xyz32(Header(stamp=node.get_clock().now().to_msg(),frame_id='odom'),source.astype(np.float32)))
        def aligned():return len(pose)==3 and np.linalg.norm(np.array(pose)-[.4,-.3,.08])<.08
        assert wait(aligned,25,publish), f'Initial alignment failed: {pose}'
        assert wait(lambda: validity and validity[-1], 2), 'Accepted localization status missing'
        print(f'PASS known transform recovered: {pose}',flush=True)
        msg=PoseWithCovarianceStamped();msg.header.frame_id='map';msg.pose.pose.position.x=.3;msg.pose.pose.position.y=-.2;msg.pose.pose.orientation.w=1.
        initial.publish(msg)
        assert wait(lambda: len(pose)==3 and abs(pose[0]-.3)<.01,3), f'Manual seed missing: {pose}'
        assert wait(aligned,20,publish), f'Manual seed was not refined: {pose}'
        print(f'PASS manual pose continues refining: {pose}',flush=True)
        msg.header.frame_id='wrong_frame';msg.pose.pose.position.x=99.;initial.publish(msg)
        wait(lambda:False,1,publish)
        assert aligned(),f'Invalid frame accepted: {pose}'
        print('PASS invalid initialpose frame rejected',flush=True)
        stop()
        wait(lambda: False, .5)
        pose.clear()
        validity.clear()
        start(args + ['-p', 'initial_max_correction_distance:=0.1'], 'startup_prior.log')
        assert wait(lambda: 'Keeping the seed;' in (OUT / 'startup_prior.log').read_text(),
                    10, publish), 'Out-of-prior candidate was not rejected'
        assert wait(lambda: len(pose)==3 and bool(validity), 2, publish)
        assert np.linalg.norm(pose) < 1e-6 and not validity[-1], (pose, validity[-1])
        print('PASS out-of-prior startup match rejected; initial pose/TF retained', flush=True)
        stop()
finally:
    stop()
    node.destroy_node()
    rclpy.shutdown()
    for f in handles:f.close()
