#!/usr/bin/env python3
"""Full Nav2/robot TF startup without LiDAR, serial, or any navigation goal."""
import os
from pathlib import Path
import signal
import subprocess
import time
import rclpy
import yaml
from lifecycle_msgs.srv import GetState
from nav_msgs.msg import Odometry
from std_msgs.msg import String, Bool
from rclpy.qos import QoSProfile, DurabilityPolicy
from tf2_ros import Buffer, TransformListener

assert os.environ.get('ROS_DOMAIN_ID') == '173'
root=Path(__file__).resolve().parents[1]
out=root/'artifacts/rviz_followup_20260924';out.mkdir(parents=True,exist_ok=True)
rclpy.init();node=rclpy.create_node('navigation_visibility_regression')
processes=[];handles=[]
def start(args,name):
    f=(out/name).open('w');handles.append(f)
    p=subprocess.Popen(args,stdout=f,stderr=subprocess.STDOUT,start_new_session=True)
    processes.append(p);return p
pub=node.create_publisher(Odometry,'aft_mapped_to_init',10)
buffer=Buffer();listener=TransformListener(buffer,node)
def tick():
    m=Odometry();m.header.stamp=node.get_clock().now().to_msg();m.header.frame_id='lidar_odom'
    m.child_frame_id='front_mid360';m.pose.pose.orientation.w=1.;pub.publish(m)
    rclpy.spin_once(node,timeout_sec=.025)
def wait(check,seconds):
    end=time.monotonic()+seconds
    while time.monotonic()<end:
        tick()
        if check():return True
    return False
try:
    start(['ros2','launch','sentry_nav_bringup','robot_state_publisher_launch.py'],'robot.log')
    start(['ros2','launch','sentry_nav_bringup','bringup_launch.py',
           f'map:={root}/src/sentry_nav_bringup/map/reality/0927.yaml',
           # Deliberately no matching map: display and startup must still work.
           'prior_pcd_file:=/tmp/nonexistent_visibility_test_map.pcd',
           f'params_file:={root}/src/sentry_nav_bringup/config/reality/nav2_params.yaml'], 'navigation.log')
    assert wait(lambda:buffer.can_transform('map','front_mid360',rclpy.time.Time()),15), 'LiDAR TF disconnected'
    assert wait(lambda:buffer.can_transform('map','gimbal_yaw_fake',rclpy.time.Time()),5), 'Navigation robot TF disconnected'
    print('PASS map -> odom -> robot/LiDAR TF before first registration',flush=True)
    descriptions=[];valid=[]
    qos=QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL)
    sub=node.create_subscription(String,'robot_description',lambda m: descriptions.append(m.data),qos)
    status=node.create_subscription(Bool,'localization_valid',lambda m:valid.append(m.data),qos)
    assert wait(lambda:descriptions and valid,5), 'Late subscribers missed model/status'
    assert '<robot' in descriptions[-1] and not valid[-1]
    cfg=yaml.safe_load((root/'src/sentry_nav_bringup/rviz/nav2_default_view.rviz').read_text())
    model=next(d for d in cfg['Visualization Manager']['Displays'] if d.get('Class')=='rviz_default_plugins/RobotModel')
    assert model['Description Topic']['Durability Policy']=='Transient Local'
    print('PASS late robot_description subscription and unlocalized status',flush=True)
    for name in ['controller_server','planner_server','bt_navigator']:
        client=node.create_client(GetState,f'/{name}/get_state')
        assert wait(client.service_is_ready,15), name+' service missing'
        state=None
        end=time.monotonic()+15
        while time.monotonic()<end:
            future=client.call_async(GetState.Request())
            assert wait(future.done,5), name+' state query timed out'
            state=future.result().current_state.label
            if state=='active':break
            wait(lambda:False,.25)
        assert state=='active',f'{name} state={state}'
        print(f'PASS {name} active with initial TF',flush=True)
finally:
    for p in reversed(processes):
        if p.poll() is None:
            p.send_signal(signal.SIGINT)
            try:p.wait(timeout=12)
            except subprocess.TimeoutExpired:os.killpg(p.pid,signal.SIGKILL);p.wait()
    for f in handles:f.close()
    node.destroy_node();rclpy.shutdown()
