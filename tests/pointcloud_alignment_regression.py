#!/usr/bin/env python3
"""Isolated, reproducible registration tests. Never publish on the robot domain.

source install/setup.bash
ROS_DOMAIN_ID=174 python3 tests/pointcloud_alignment_regression.py
Optional --real-dir supplies prior.npy and before/registered_scan.npy from a capture.
Results and node logs go under logs/alignment_regression/.
"""
import argparse
import json
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
from rclpy.qos import QoSProfile, DurabilityPolicy
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py.point_cloud2 import create_cloud_xyz32
from std_msgs.msg import Header, Bool
from tf2_msgs.msg import TFMessage
from tf2_ros import StaticTransformBroadcaster

ROOT = Path(__file__).resolve().parents[1]
OUT = ROOT / 'logs/alignment_regression'
assert os.environ.get('ROS_DOMAIN_ID') == '174', 'Use isolated ROS_DOMAIN_ID=174'
parser = argparse.ArgumentParser()
parser.add_argument('--real-dir', type=Path)
parser.add_argument('--test-zero-seed', action='store_true',
                    help='Also test a captured known-origin fixture from a zero startup pose')
parser.add_argument('--heading-search', action='store_true')
parser.add_argument('--output-dir', type=Path, default=OUT)
args = parser.parse_args()
OUT = args.output_dir
OUT.mkdir(parents=True, exist_ok=True)
rclpy.init()
node = rclpy.create_node('pointcloud_alignment_regression')
qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
results = []


def rotate(points, yaw):
    c, s = math.cos(yaw), math.sin(yaw)
    return points @ np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]]).T


def wrap(yaw):
    return math.atan2(math.sin(yaw), math.cos(yaw))


def save_pcd(path, points):
    header = ('VERSION .7\nFIELDS x y z\nSIZE 4 4 4\nTYPE F F F\nCOUNT 1 1 1\n'
              f'WIDTH {len(points)}\nHEIGHT 1\nPOINTS {len(points)}\nDATA binary\n')
    with path.open('wb') as f:
        f.write(header.encode())
        f.write(np.asarray(points, dtype=np.float32).tobytes())


def scene(seed):
    rng = np.random.default_rng(seed)
    walls = []
    for _ in range(8):
        origin = rng.uniform(-4, 4, 2)
        angle = rng.uniform(-math.pi, math.pi)
        length = rng.uniform(.6, 2.)
        along = rng.uniform(-length, length, 500)
        walls.append(np.c_[origin[0]+along*np.cos(angle), origin[1]+along*np.sin(angle),
                           rng.uniform(.25, 1.4, 500)])
    return np.vstack(walls).astype(np.float32)


def run_case(name, target, source, truth, seed, negative=False, manual=False,
             dropout=False, wrong_frame=False, tolerance=.08, freshness=False, all_yaws=False):
    namespace = '/' + name
    poses, valid = [], []
    def on_tf(msg):
        for t in msg.transforms:
            if t.header.frame_id == name+'_map' and t.child_frame_id == name+'_odom':
                v = t.transform
                poses.append([v.translation.x, v.translation.y,
                              2*math.atan2(v.rotation.z, v.rotation.w)])
    subscriptions = [node.create_subscription(TFMessage, namespace+'/tf', on_tf, 20),
                     node.create_subscription(Bool, namespace+'/localization_valid',
                                              lambda m: valid.append(m.data), qos)]
    cloud_pub = node.create_publisher(PointCloud2, namespace+'/registered_scan', 5)
    pose_pub = node.create_publisher(PoseWithCovarianceStamped, namespace+'/initialpose', 5)
    broadcaster = StaticTransformBroadcaster(node)
    tf = TransformStamped()
    tf.header.frame_id = name+'_odom'; tf.child_frame_id = name+'_base'
    tf.transform.rotation.w = 1.
    broadcaster.sendTransform(tf)
    path = OUT/(name+'.pcd')
    save_pcd(path, target)
    params = dict(prior_pcd_file=str(path), map_frame=name+'_map', odom_frame=name+'_odom',
                  base_frame=name+'_base', registration_height_filter=True,
                  global_leaf_size=.1, registered_leaf_size=.1, accumulated_count_threshold=4,
                  max_iterations=80, min_range=0., max_fitness_error=.3,
                  enable_periodic_relocalization=True, relocalization_interval=2.0 if freshness else .3,
                  max_correction_yaw=.2, max_correction_distance=.5,
                  initial_max_correction_distance=1., initial_max_correction_yaw=.35,
                  initial_search_all_yaws=all_yaws,
                  geometric_max_distance=.15, geometric_min_overlap=.8,
                  localization_valid_timeout=4.0 if freshness else 1.5,
                  init_pose=[float(seed[0]), float(seed[1]), 0., 0., 0., float(seed[2])])
    command = ['ros2','run','small_gicp_relocalization','small_gicp_relocalization_node',
               '--ros-args','-r','__ns:='+namespace,'-r','/tf:='+namespace+'/tf']
    for k,v in params.items():
        command += ['-p', k+':='+str(v).lower() if isinstance(v,bool) else k+':='+str(v)]
    log = (OUT/(name+'.log')).open('w')
    process = subprocess.Popen(command, stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
    started = time.monotonic()
    def spin(seconds, publish=False):
        until = time.monotonic()+seconds
        next_pub = 0.
        while time.monotonic()<until:
            if publish and time.monotonic()>next_pub:
                cloud_pub.publish(create_cloud_xyz32(Header(stamp=node.get_clock().now().to_msg(),
                    frame_id='wrong_frame' if wrong_frame else name+'_odom'), source.astype(np.float32)))
                next_pub = time.monotonic()+.04
            rclpy.spin_once(node, timeout_sec=.01)
            assert process.poll() is None, 'localizer exited'
    def aligned():
        return (poses and valid and valid[-1] and
                np.linalg.norm(np.array(poses[-1][:2])-truth[:2])<tolerance and
                abs(wrap(poses[-1][2]-truth[2]))<.04)
    try:
        while cloud_pub.get_subscription_count()==0 and time.monotonic()-started<8: spin(.05)
        assert cloud_pub.get_subscription_count(), 'no subscription'
        spin(.2)
        assert not any(valid), 'seed TF falsely marked valid'
        if manual:
            m=PoseWithCovarianceStamped();m.header.frame_id=name+'_map'
            m.pose.pose.position.x=float(truth[0]-.04);m.pose.pose.position.y=float(truth[1]+.03)
            m.pose.pose.orientation.z=math.sin((truth[2]-.03)/2)
            m.pose.pose.orientation.w=math.cos((truth[2]-.03)/2)
            pose_pub.publish(m);spin(.2)
            assert not any(valid), 'manual unverified seed falsely marked valid'
        if negative:
            spin(12.0 if all_yaws else 2.5,publish=True)
            assert not any(valid), 'bad input accepted as localized'
        else:
            deadline=time.monotonic()+8
            while not aligned() and time.monotonic()<deadline:spin(.1,publish=True)
            assert aligned(), f'wrong solution {poses[-1:]}, truth={truth}, valid={valid[-1:]}'
            # Several periodic matches must not walk away from the known pose.
            spin(1.1,publish=True)
            assert aligned(), f'periodic drift {poses[-1:]}'
            if freshness:
                # The old implementation froze the first complete batch for 2s.
                # Change observations after that batch fills but before the timer.
                source = source - np.array([.25, 0., 0.])
                truth = np.array(truth, copy=True)
                truth[:2] += rotate(np.array([[.25, 0., 0.]]), truth[2])[0,:2]
                spin(1.2, publish=True)
                assert aligned(), f'periodic registration used stale scans: {poses[-1:]}'
            if dropout:
                spin(2.)
                assert valid and not valid[-1], 'sensor dropout left localization_valid=true'
                deadline=time.monotonic()+4
                while not aligned() and time.monotonic()<deadline:spin(.1,publish=True)
                assert aligned(), 'did not recover after sensor resumed'
        result=dict(name=name,passed=True,seconds=round(time.monotonic()-started,2),
                    final_pose=poses[-1] if poses else None,negative=negative)
    except Exception as exc:
        result=dict(name=name,passed=False,error=str(exc),final_pose=poses[-1:] or None)
    finally:
        if process.poll() is None:os.killpg(process.pid,signal.SIGINT)
        try:process.wait(timeout=8)
        except subprocess.TimeoutExpired:os.killpg(process.pid,signal.SIGKILL);process.wait()
        log.close();path.unlink()
        for sub in subscriptions:node.destroy_subscription(sub)
        node.destroy_publisher(cloud_pub);node.destroy_publisher(pose_pub)
    results.append(result)
    (OUT/'results.json').write_text(json.dumps(results,indent=2))
    print(json.dumps(result),flush=True)


try:
    if args.heading_search:
        target=scene(100)
        for i,yaw in enumerate([.9,-1.7,2.8]):
            truth=np.array([.12,-.08,yaw]);source=rotate(target-np.r_[truth[:2],0.],-yaw)
            run_case('unknown_heading_'+str(i),target,source,truth,[0,0,0],all_yaws=True)
        symmetric=np.vstack([target,rotate(target,math.pi)])
        run_case('ambiguous_heading',symmetric,target,[0,0,0],[0,0,0],negative=True,all_yaws=True)
        run_case('heading_search_wrong_map',scene(987),target,[0,0,0],[0,0,0],negative=True,all_yaws=True)
    for map_index in range(3):
        target=scene(100+map_index)
        for i,yaw in enumerate([0.,.6,1.9,-2.7]):
            truth=np.array([map_index*1.1, -.7*map_index,yaw])
            source=rotate(target-np.r_[truth[:2],0.],-yaw)
            rng=np.random.default_rng(300+map_index*10+i)
            source += rng.normal(0,.008,source.shape)
            # Floors/ceilings and missing returns differ between scans/maps.
            source=source[rng.random(len(source))>.25]
            horizontal=np.c_[rng.uniform(-7,7,(2000,2)),rng.choice([0.,3.],2000)]
            run_case(f'map{map_index}_yaw{i}',np.vstack([target,horizontal]),
                     np.vstack([source,horizontal]),truth,truth+[.15,-.1,.07],dropout=i==0)
    target=scene(100)
    truth=np.array([.4,-.3,.08]); source=rotate(target-np.r_[truth[:2],0.],-truth[2])
    run_case('latest_scan_window',target,source,truth,[0,0,0],freshness=True)
    run_case('manual_new_position',target,source,truth,[0,0,0],manual=True)
    low=target.copy();low[:,2]=.4
    high=low.copy();high[:,2]=1.1
    run_case('wrong_height_same_xy',low,high,[0,0,0],[0,0,0],negative=True)
    run_case('wrong_map',scene(987),source,truth,[0,0,0],negative=True)
    run_case('wrong_heading',target,rotate(target,1.7),[0,0,-1.7],[0,0,0],negative=True)
    run_case('wrong_frame',target,source,truth,[0,0,0],negative=True,wrong_frame=True)
    run_case('empty',target,np.empty((0,3)),truth,[0,0,0],negative=True)
    run_case('nonfinite',target,np.full((100,3),np.nan),truth,[0,0,0],negative=True)
    run_case('too_sparse',target,source[:12],truth,[0,0,0],negative=True)
    if args.real_dir:
        prior=np.load(args.real_dir/'prior.npy')
        observed=np.load(args.real_dir/'before/registered_scan.npy')[::5]
        # Recorded reference enables frame-equivariance checks; not ground-truth robot pose.
        truth=np.asarray(json.loads((args.real_dir/'reference_pose.json').read_text())['pose'])
        if args.test_zero_seed:
            run_case('real_known_origin_zero_seed',prior,observed,truth,[0,0,0],tolerance=.15,all_yaws=args.heading_search)
        wrong_seed=truth.copy();wrong_seed[2]=wrap(truth[2]+1.5)
        run_case('real_wrong_heading_seed',prior,observed,truth,wrong_seed,negative=True)
        for i,yaw in enumerate([0.,.5,-1.2,2.8]):
            shift=np.array([i*.2,-i*.1,0.]); shifted=rotate(observed,yaw)+shift
            new_yaw=wrap(truth[2]-yaw)
            new_t=np.r_[truth[:2],0.]-rotate(shift[None,:],new_yaw)[0]
            new_truth=np.r_[new_t[:2],new_yaw]
            run_case('real_frame_'+str(i),prior,shifted,new_truth,
                     new_truth+[.03,-.03,.02],tolerance=.15)
finally:
    node.destroy_node();rclpy.shutdown()
failed=[r for r in results if not r['passed']]
print(f'{len(results)-len(failed)}/{len(results)} cases passed',flush=True)
raise SystemExit(bool(failed))
