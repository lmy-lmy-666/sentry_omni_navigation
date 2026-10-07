#!/usr/bin/env python3
"""Launch-level map/profile isolation checks; ROS_DOMAIN_ID=175 only."""
import json,os,signal,subprocess,tempfile,time
from pathlib import Path
import rclpy,yaml
from rcl_interfaces.srv import GetParameters

assert os.environ.get('ROS_DOMAIN_ID')=='175'
ROOT=Path(__file__).resolve().parents[1];OUT=ROOT/'logs/map_profile_regression';OUT.mkdir(exist_ok=True,parents=True)
rclpy.init();node=rclpy.create_node('map_profile_test');results=[]
with tempfile.TemporaryDirectory() as tmp:
 custom=Path(tmp)/'custom.yaml';custom.write_text('/**:\n  ros__parameters:\n    init_pose: [1.0, 2.0, 0.0, 0.0, 0.0, -0.7]\n')
 cases=[('0927','auto',0.,False),('new_site','auto',0.,False),('0927','none',0.,False),
        ('new_site',str(custom),-.7,False),('0927','auto',0.,True)]
 for index,(world,profile,yaw,namespaced) in enumerate(cases):
  namespace=f'profile_case_{index}' if namespaced else ''
  log=(OUT/f'case{index}.log').open('w')
  # A nonexistent PCD is intentional: profile loading must not require LiDAR or a large map.
  command=['ros2','launch','sentry_nav_bringup','localization_launch.py',
           f'map:={tmp}/{world}.yaml','prior_pcd_file:=/tmp/no_profile_test_map.pcd',
           f'params_file:={ROOT}/src/sentry_nav_bringup/config/reality/nav2_params.yaml',
           'autostart:=False','use_composition:=False',
           'localization_params_file:='+profile]
  if namespace:command.append('namespace:='+namespace)
  p=subprocess.Popen(command,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
  client=node.create_client(GetParameters,f'/{namespace+"/" if namespace else ""}small_gicp_relocalization/get_parameters')
  try:
   end=time.monotonic()+10
   while not client.service_is_ready() and time.monotonic()<end:rclpy.spin_once(node,timeout_sec=.1)
   assert client.service_is_ready(),'parameter service unavailable'
   future=client.call_async(GetParameters.Request(names=['init_pose']))
   while not future.done() and time.monotonic()<end:rclpy.spin_once(node,timeout_sec=.1)
   actual=list(future.result().values[0].double_array_value)
   assert len(actual)==6 and abs(actual[-1]-yaw)<1e-9,(world,profile,actual)
   result=dict(case=index,map=world,profile=profile,namespace=namespace,init_pose=actual,passed=True)
   print(json.dumps(result),flush=True);results.append(result)
  finally:
   if p.poll() is None:os.killpg(p.pid,signal.SIGINT)
   try:p.wait(timeout=10)
   except subprocess.TimeoutExpired:os.killpg(p.pid,signal.SIGKILL);p.wait()
   log.close();node.destroy_client(client)
   end=time.monotonic()+.5
   while time.monotonic()<end:rclpy.spin_once(node,timeout_sec=.05)
(OUT/'results.json').write_text(json.dumps(results,indent=2))
node.destroy_node();rclpy.shutdown()
