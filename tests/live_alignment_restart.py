#!/usr/bin/env python3
"""Stationary robot restart acceptance test. Starts navigation with decision disabled.
Requires explicit --stop-launch-pid to stop an existing launch; never sends goals.
Leaves the final navigation launch running and prints its PID. Live hardware required.
"""
import argparse,json,math,os,signal,subprocess,time
from pathlib import Path
import numpy as np
import rclpy
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2
from sensor_msgs.msg import JointState
from geometry_msgs.msg import TransformStamped
from rclpy.qos import QoSProfile, DurabilityPolicy
from sensor_msgs_py.point_cloud2 import read_points_numpy
from scipy.spatial import cKDTree
from scipy.spatial.transform import Rotation
from std_msgs.msg import Bool
from tf2_ros import Buffer, TransformListener

parser=argparse.ArgumentParser()
parser.add_argument('--world',required=True)
parser.add_argument('--stop-launch-pid',type=int)
parser.add_argument('--restarts',type=int,default=5)
parser.add_argument('--duration',type=float,default=25.)
parser.add_argument('--output-dir',default='logs/live_alignment_restart')
args=parser.parse_args()
assert os.environ.get('ROS_DOMAIN_ID','0')=='0', 'This is the live robot domain test'
ROOT=Path(__file__).resolve().parents[1]
OUT=ROOT/args.output_dir;OUT.mkdir(parents=True,exist_ok=True)
# This workspace's saved Point-LIO files use float32 binary records.
f=(ROOT/f'src/sentry_nav_bringup/pcd/reality/{args.world}.pcd').open('rb');header={}
while True:
 line=f.readline().decode().strip()
 if line and not line.startswith('#'):
  key,*vals=line.split();header[key]=vals
  if key=='DATA':break
assert header['DATA']==['binary'] and set(header['SIZE'])=={'4'} and set(header['TYPE'])=={'F'}
points=np.fromfile(f,dtype=np.float32).reshape(-1,len(header['FIELDS']))[::20,:3];f.close()
points=points[np.all(np.isfinite(points),axis=1)&(points[:,2]>.2)&(points[:,2]<1.5)]
tree=cKDTree(points)
if args.stop_launch_pid:
 os.kill(args.stop_launch_pid,signal.SIGINT)
 end=time.monotonic()+20
 while Path(f'/proc/{args.stop_launch_pid}').exists() and time.monotonic()<end:time.sleep(.2)
 assert not Path(f'/proc/{args.stop_launch_pid}').exists(),'Existing launch did not stop'
rclpy.init();results=[];process=None
for run in range(args.restarts):
 if process:
  process.send_signal(signal.SIGINT)
  try:process.wait(timeout=15)
  except subprocess.TimeoutExpired:os.killpg(process.pid,signal.SIGKILL);process.wait()
  log.close();time.sleep(1)
 log=(OUT/f'run{run+1}.log').open('w')
 process=subprocess.Popen(['ros2','launch','sentry_nav_bringup','rm_sentry_launch.py',
   'world:='+args.world,'enable_behavior:=False'],stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
 node=rclpy.create_node(f'restart_alignment_observer_{run}')
 buffer=Buffer();listener=TransformListener(buffer,node)
 state={'valid':False,'frame':0,'samples':[],'last_sample':0.,'errors':[], 'startup':{}, 'valid_after_start':[],'localized_once':False}
 def record_startup(msg,key):
  if key in state['startup']:return
  if isinstance(msg,JointState):
   state['startup'][key]={'names':list(msg.name),'position':list(msg.position)}
  else:
   v=msg.transform;t=v.translation;q=v.rotation
   state['startup'][key]=[t.x,t.y,t.z,q.x,q.y,q.z,q.w]
 node.create_subscription(JointState,'serial/gimbal_joint_state',lambda m:record_startup(m,'serial_joints'),10)
 node.create_subscription(JointState,'joint_states',lambda m:record_startup(m,'published_joints'),10)
 node.create_subscription(TransformStamped,'odom_to_lidar_odom',lambda m:record_startup(m,'odom_to_lidar_odom'),QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL))
 def validity(msg):
  state['valid']=msg.data
  state['localized_once']=state['localized_once'] or msg.data
  if state['localized_once']:state['valid_after_start'].append(msg.data)
 node.create_subscription(Bool,'localization_valid',validity,10)
 def cloud(msg):
  state['frame']+=1
  now=time.monotonic()
  if not state['valid'] or now-state['last_sample']<1.:return
  try:
   # registered_scan already lives in odom; use the same map correction as RViz.
   tf=buffer.lookup_transform('map',msg.header.frame_id,rclpy.time.Time()).transform
   a=read_points_numpy(msg,field_names=['x','y','z'],skip_nans=True)
   a=a[(a[:,2]>.2)&(a[:,2]<1.5)]
   if len(a)<30:return
   q=tf.rotation;t=tf.translation
   a=Rotation.from_quat([q.x,q.y,q.z,q.w]).apply(a)+[t.x,t.y,t.z]
   distances,_=tree.query(a)
   if np.median(distances)>=.07 or np.quantile(distances,.9)>=.2 or np.mean(distances<.15)<=.8:
    np.save(OUT/f'run{run+1}_bad_{now-started:.2f}.npy',a)
   state['samples'].append({'seconds':round(now-started,2),'median':float(np.median(distances)),
    'p90':float(np.quantile(distances,.9)),'overlap15':float(np.mean(distances<.15)),
    'points':len(a),'map_to_odom':[t.x,t.y,t.z,q.x,q.y,q.z,q.w]})
   state['last_sample']=now
  except Exception as exc:state['errors'].append(str(exc))
 node.create_subscription(PointCloud2,'registered_scan',cloud,qos_profile_sensor_data)
 started=time.monotonic()
 while time.monotonic()-started<args.duration and process.poll() is None:
  rclpy.spin_once(node,timeout_sec=.05)
 samples=state['samples']
 result={'run':run+1,'pid':process.pid,'samples':samples,'received_frames':state['frame'],
   'passed':bool(len(samples)>=10 and all(state['valid_after_start']) and all(s['median']<.07 and s['p90']<.2 and s['overlap15']>.8 for s in samples)),
   'invalid_after_first_localization':state['valid_after_start'].count(False),
   'tf_errors':len(state['errors']),'startup':state['startup']}
 results.append(result);(OUT/'results.json').write_text(json.dumps(results,indent=2))
 print(json.dumps({k:v for k,v in result.items() if k!='samples'}),flush=True)
 node.destroy_node()
print('Final navigation left running, PID',process.pid,flush=True)
rclpy.shutdown()
raise SystemExit(not all(r['passed'] for r in results))
