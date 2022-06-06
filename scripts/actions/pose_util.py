#!/usr/bin/python3

import tf
from std_msgs.msg import Header
from geometry_msgs.msg import Quaternion, Vector3, Pose, PoseStamped
from nav_msgs.msg import Path
import math

import rospy

# >>> euler_to_quarternion(Vector3(0.0, 0.0, math.pi / 2.0))
# x: 0.0
# y: 0.0
# z: 0.707106781187
# w: 0.707106781187

def euler_to_quarternion(euler):
    """Convert Euler Angles to Quaternion
    euler: geometry_msgs/Vector3
    quaternion: geometry_msgs/Quaternion
    """
    q = tf.transformations.quaternion_from_euler(euler.x, euler.y, euler.z)
    return Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])

# >>> angle2dToQuat(math.pi/2)
# x: 0.0
# y: 0.0
# z: 0.707106781187
# w: 0.707106781187

def angle2dToQuat(angle):
    return euler_to_quarternion(Vector3(0.0, 0.0, angle))

def ROS2DPose(x,y,angle,z=0):

    p = Pose()
    p.position.x = x
    p.position.y = y
    p.position.z = z
    p.orientation = angle2dToQuat(angle)

    return p


def ROS2DPoseStamped(x, y, angle, frame_id="map", z=0):

    h = Header()
    h.frame_id = frame_id
    p = ROS2DPose(x,y,angle)

    ps = PoseStamped()
    ps.header = h
    ps.pose = p

    return ps

def ROS2DPath(ros_posestamps, frame_id="map"):

    h = Header()
    h.frame_id = frame_id

    path = Path()
    path.header = h
    path.poses = ros_posestamps

    return path
   
# >>> quaternion_to_euler(Quaternion(0.0, 0.0, 0.7071, 0.7071))
# x: 0.0
# y: -0.0
# z: 1.57079632679

def quaternion_to_euler(quaternion):
    """Convert Quaternion to Euler Angles
    quarternion: geometry_msgs/Quaternion
    euler: geometry_msgs/Vector3
    """
    e = tf.transformations.euler_from_quaternion((quaternion.x, quaternion.y, quaternion.z, quaternion.w))
    return Vector3(x=e[0], y=e[1], z=e[2])

def quaternion_to_2d_vector(quaternion):
    ang = quaternion_to_euler(quaternion).z
    return [math.cos(ang), math.sin(ang)]

if __name__ == '__main__':

  #print(euler_to_quarternion(Vector3(0.0, 0.0, math.pi / 2.0)))
  #print(quaternion_to_euler(Quaternion(0.0, 0.0, 0.7071, 0.7071)))
  print(quaternion_to_2d_vector(Quaternion(0.0, 0.0, 0.7071, 0.7071)))
  #print(angle2dToQuat(math.pi/2))
  #print(ROS2DPose(0,0,math.pi/2))

  rospy.init_node('ros_pose_utils', anonymous=True)
  pose_pub = rospy.Publisher('/ros_pose_utils/pose', PoseStamped, queue_size=10)
  pose_pub2 = rospy.Publisher('/ros_pose_utils/pose2', PoseStamped, queue_size=10)
  pose_pub3 = rospy.Publisher('/ros_pose_utils/pose3', PoseStamped, queue_size=10)
  path_pub = rospy.Publisher('/ros_pose_utils/path', Path, queue_size=10)
  r = rospy.Rate(10)
  
  i = 0

  while not rospy.is_shutdown():
    pose_pub.publish(ROS2DPoseStamped(i % 10 - 5, 0, 0))
    pose_pub2.publish(ROS2DPoseStamped(-3, i % 10 - 5, math.pi/2))
    pose_pub3.publish(ROS2DPoseStamped(3, 3, math.pi/10 * i))

    p1 = ROS2DPoseStamped(1, -2.5, 0)
    p2 = ROS2DPoseStamped(4, -2.5, 0)

    path = ROS2DPath([p1,p2]) 
    path_pub.publish(path)

    r.sleep()
    i = i+1
