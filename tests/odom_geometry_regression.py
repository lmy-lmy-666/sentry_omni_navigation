#!/usr/bin/env python3
"""Check bridge frame algebra with rotated/tilted mounts and delayed scans."""
import os,signal,subprocess,time,json
from pathlib import Path
import numpy as np
import rclpy
from scipy.spatial.transform import Rotation
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py.point_cloud2 import create_cloud_xyz32,read_points_numpy
from std_msgs.msg import Header
from tf2_ros import StaticTransformBroadcaster
from rclpy.qos import QoSProfile, DurabilityPolicy

assert os.environ.get('ROS_DOMAIN_ID')=='176'
ROOT=Path(__file__).resolve().parents[1];OUT=ROOT/'logs/odom_geometry_regression';OUT.mkdir(exist_ok=True,parents=True)
rclpy.init();node=rclpy.create_node('odom_geometry_regression');results=[]
def matrix(xyz,rpy):
 t=np.eye(4);t[:3,:3]=Rotation.from_euler('xyz',rpy).as_matrix();t[:3,3]=xyz;return t
def transform(points,t):return points@t[:3,:3].T+t[:3,3]
for i,(yaw,pitch) in enumerate([(0,0),(.7,0),(3.14,.524),(-2.,-.4),(1.9,.524),(-.8,.3)]):
 ns=f'/odom_case_{i}';base=f'base_{i}';lidar=f'lidar_{i}'
 mount=matrix([-.15,.02,.433],[0,pitch,yaw]);initial=matrix([.2,-.4,.1],[.6,-.2,1.3])
 odom_to_lio=mount@np.linalg.inv(initial)
 tf=TransformStamped();tf.header.frame_id=base;tf.child_frame_id=lidar
 tf.transform.translation.x,tf.transform.translation.y,tf.transform.translation.z=mount[:3,3]
 q=Rotation.from_matrix(mount[:3,:3]).as_quat();tf.transform.rotation.x,tf.transform.rotation.y,tf.transform.rotation.z,tf.transform.rotation.w=q
 broadcaster=StaticTransformBroadcaster(node);broadcaster.sendTransform(tf)
 pub=node.create_publisher(Odometry,ns+'/aft_mapped_to_init',10)
 clouds=node.create_publisher(PointCloud2,ns+'/cloud_registered',10)
 received={}
 offsets=[]
 offset_sub=node.create_subscription(TransformStamped,ns+'/odom_to_lidar_odom',offsets.append,
   QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL))
 subs=[node.create_subscription(PointCloud2,ns+'/'+name,lambda m,name=name:received.update({name:m}),10)
       for name in ['sensor_scan','registered_scan']]
 log=(OUT/f'case{i}.log').open('w');p=subprocess.Popen(['ros2','run','odom_bridge','odom_bridge_node','--ros-args','-r','__ns:='+ns,'-p','base_frame:='+base,'-p','lidar_frame:='+lidar,'-p','robot_base_frame:='+base,'-p','cf_enabled:=false'],stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
 def spin(seconds):
  end=time.monotonic()+seconds
  while time.monotonic()<end:rclpy.spin_once(node,timeout_sec=.01)
 def odom(t,stamp):
  m=Odometry();m.header.stamp=stamp;m.header.frame_id='camera_init'
  m.pose.pose.position.x,m.pose.pose.position.y,m.pose.pose.position.z=t[:3,3]
  q=Rotation.from_matrix(t[:3,:3]).as_quat();m.pose.pose.orientation.x,m.pose.pose.orientation.y,m.pose.pose.orientation.z,m.pose.pose.orientation.w=q
  pub.publish(m)
 try:
  end=time.monotonic()+6
  while pub.get_subscription_count()<2 and time.monotonic()<end:spin(.1)
  assert pub.get_subscription_count()>=2,'bridge not discovered'
  end=time.monotonic()+3
  while not offsets and time.monotonic()<end:
   odom(initial,node.get_clock().now().to_msg());spin(.05)
  assert offsets,'initialization transform missing'
  off=offsets[-1].transform
  actual_offset=np.eye(4);actual_offset[:3,:3]=Rotation.from_quat([off.rotation.x,off.rotation.y,off.rotation.z,off.rotation.w]).as_matrix()
  actual_offset[:3,3]=[off.translation.x,off.translation.y,off.translation.z]
  assert np.allclose(actual_offset,odom_to_lio,atol=1e-6),('initial offset mismatch',actual_offset,odom_to_lio)
  pose=matrix([1.2,-.7,.1],[.6,-.2,1.6]);stamp=node.get_clock().now().to_msg();odom(pose,stamp);spin(.1)
  odom(matrix([2.2,-.7,.1],[.6,-.2,2.]),node.get_clock().now().to_msg());spin(.1)
  points=np.array([[5.,2.,.8],[-2.,3.,1.2],[.4,-1.,.2]],dtype=np.float32)
  clouds.publish(create_cloud_xyz32(Header(stamp=stamp,frame_id='camera_init'),points));spin(.5)
  for name,expected in [('sensor_scan',transform(points,np.linalg.inv(pose))),('registered_scan',transform(points,odom_to_lio))]:
   assert name in received,'missing '+name
   actual=read_points_numpy(received[name],field_names=['x','y','z'])
   error=float(np.max(np.abs(actual-expected)));assert error<1e-5,(name,error)
   assert received[name].header.stamp==stamp,'scan timestamp changed'
  result=dict(case=i,yaw=yaw,pitch=pitch,passed=True,max_error=error);results.append(result);print(json.dumps(result),flush=True)
 finally:
  if p.poll() is None:os.killpg(p.pid,signal.SIGINT)
  p.wait(timeout=10);log.close()
  node.destroy_publisher(pub);node.destroy_publisher(clouds)
  node.destroy_subscription(offset_sub)
  for sub in subs:node.destroy_subscription(sub)
(OUT/'results.json').write_text(json.dumps(results,indent=2));node.destroy_node();rclpy.shutdown()
