#!/usr/bin/env python3
"""Start only terrain nodes and LiDAR driver; no serial or navigation control."""
import os
from pathlib import Path
import signal
import subprocess
import time
import rclpy
from sensor_msgs.msg import PointCloud2
from rclpy.qos import qos_profile_sensor_data
assert os.environ.get('ROS_DOMAIN_ID')=='173'
root=Path(__file__).resolve().parents[1]
out=root/'artifacts/rviz_followup_20260924'
rclpy.init();node=rclpy.create_node('sensor_shutdown_regression')
clouds=[]
sub=node.create_subscription(PointCloud2,'livox/lidar/pointcloud',clouds.append,qos_profile_sensor_data)
try:
 for package,exe,args,seconds in [
  ('terrain_analysis','terrainAnalysis',[],1.),
  ('terrain_analysis_ext','terrainAnalysisExt',[],1.),
  ('livox_ros_driver2','livox_ros_driver2_node',['--ros-args','-p','xfer_format:=4','-p',f'user_config_path:={root}/src/sentry_nav_bringup/config/reality/mid360_user_config.json'],6.)]:
  with (out/(package+'_shutdown.log')).open('w') as log:
   p=subprocess.Popen([str(root/'install'/package/'lib'/package/exe)]+args,stdout=log,stderr=subprocess.STDOUT)
   try:
    end=time.monotonic()+seconds
    while time.monotonic()<end:rclpy.spin_once(node,timeout_sec=.05)
    assert p.poll() is None, f'{package} exited before SIGINT'
    p.send_signal(signal.SIGINT)
    code=p.wait(timeout=10)
    assert code==0,f'{package} exited with {code}'
    print(f'PASS {package}: SIGINT exit=0',flush=True)
   finally:
    if p.poll() is None:p.kill();p.wait()
 print(f'LiDAR smoke test received {len(clouds)} live PointCloud2 messages',flush=True)
finally:
 node.destroy_node();rclpy.shutdown()
