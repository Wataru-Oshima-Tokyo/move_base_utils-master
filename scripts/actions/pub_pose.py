import tf
from geometry_msgs.msg import Quaternion, Vector3, Pose
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

if __name__ == '__main__':

  #print(euler_to_quarternion(Vector3(0.0, 0.0, math.pi / 2.0)))
  #print(quaternion_to_euler(Quaternion(0.0, 0.0, 0.7071, 0.7071)))
  #print(angle2dToQuat(math.pi/2))
  #print(ROS2DPose(0,0,math.pi/2))

  rospy.init_node('ros_pose_utils', anonymous=True)
  pose_pub = rospy.Publisher('/pose_utils/pose', Pose, queue_size=10)
  r = rospy.Rate(1)

  while not rospy.is_shutdown():
    pose_pub.publish(ROS2DPose(0,0,0))
    r.sleep()
