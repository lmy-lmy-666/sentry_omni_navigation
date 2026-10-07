#!/usr/bin/env python3
"""Save a small live alignment fixture to a persistent directory for regression replay.
Reference pose is the running localizer's estimate, not surveyed ground truth.
"""
import argparse,json,time
from pathlib import Path
import numpy as np
import rclpy
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py.point_cloud2 import read_points_numpy
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import Bool
from tf2_ros import Buffer,TransformListener

p=argparse.ArgumentParser();p.add_argument('--pcd',type=Path,required=True);p.add_argument('--out',type=Path,required=True);args=p.parse_args()
args.out.mkdir(exist_ok=True,parents=True);(args.out/'before').mkdir(exist_ok=True)
with args.pcd.open('rb') as f:
 header={}
 while True:
  line=f.readline().decode().strip()
  if line and not line.startswith('#'):
   key,*v=line.split();header[key]=v
   if key=='DATA':break
 assert header['DATA']==['binary'] and set(header['SIZE'])=={'4'} and set(header['TYPE'])=={'F'}
 a=np.fromfile(f,dtype=np.float32).reshape(-1,len(header['FIELDS']))[::20,:3]
 np.save(args.out/'prior.npy',a)
rclpy.init();node=rclpy.create_node('capture_alignment_fixture');b=Buffer();l=TransformListener(b,node)
clouds=[];valid=[False];first=[None]
node.create_subscription(Bool,'localization_valid',lambda m:valid.__setitem__(0,m.data),10)
def callback(m):
 if valid[0] and b.can_transform('map','odom',rclpy.time.Time()):
  clouds.append(read_points_numpy(m,field_names=['x','y','z'],skip_nans=True))
  if first[0] is None:first[0]=time.monotonic()
node.create_subscription(PointCloud2,'registered_scan',callback,qos_profile_sensor_data)
end=time.monotonic()+15
while time.monotonic()<end:
 rclpy.spin_once(node,timeout_sec=.02)
 if first[0] is not None and time.monotonic()-first[0]>1.:break
assert len(clouds)>10,'Not enough valid live scans'
t=b.lookup_transform('map','odom',rclpy.time.Time()).transform
ref=[t.translation.x,t.translation.y,2*np.arctan2(t.rotation.z,t.rotation.w)]
np.save(args.out/'before/registered_scan.npy',np.concatenate(clouds))
(args.out/'reference_pose.json').write_text(json.dumps({'pose':ref,'note':'Live localizer estimate for metamorphic tests; not external ground truth'},indent=2))
print('Saved',len(clouds),'frames, reference pose',ref,'to',args.out)
node.destroy_node();rclpy.shutdown()
