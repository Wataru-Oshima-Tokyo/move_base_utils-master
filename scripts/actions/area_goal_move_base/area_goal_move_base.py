#!/usr/bin/env python

import rospy
import actionlib
import tf
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion
import math
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from math import pi
import os

from visualization_msgs.msg import Marker

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

def goal_pose(pose,frame):
    goal_pose = MoveBaseGoal()
    goal_pose.target_pose.header.frame_id = frame
    goal_pose.target_pose.pose.position.x = pose[0][0]
    goal_pose.target_pose.pose.position.y = pose[0][1]
    goal_pose.target_pose.pose.position.z = pose[0][2]
    goal_pose.target_pose.pose.orientation.x = pose[1][0]
    goal_pose.target_pose.pose.orientation.y = pose[1][1]
    goal_pose.target_pose.pose.orientation.z = pose[1][2]
    goal_pose.target_pose.pose.orientation.w = pose[1][3]

    return goal_pose

def non_marker(frame):
    marker_data = Marker()
    marker_data.header.frame_id = frame
    marker_data.ns = "area_goal_move_base"
    marker_data.id = 0
    return marker_data

def marker(x,y,frame,radius,gray=0):
    # Mark arrow
    marker_data = Marker()
    marker_data.header.frame_id = frame
    marker_data.header.stamp = rospy.Time.now()

    marker_data.ns = "basic_shapes"
    marker_data.id = 0

    marker_data.action = Marker.ADD

    marker_data.pose.position.x = x    
    marker_data.pose.position.y = y
    marker_data.pose.position.z = 0

    if gray:

      marker_data.color.r = 0.3
      marker_data.color.g = 0.3
      marker_data.color.b = 0.3

    else:

      marker_data.color.r = 1.0
      marker_data.color.g = 0.0
      marker_data.color.b = 0.0


    marker_data.color.a = 0.5
    marker_data.scale.x = radius * 2
    marker_data.scale.y = radius * 2
    marker_data.scale.z = 0.01

    marker_data.lifetime = rospy.Duration(0.2)

    marker_data.type = 3

    return marker_data

def get_distance(p1, p2):
    d = math.sqrt((p2[0] - p1[0]) ** 2 + (p2[1] - p1[1]) ** 2)
    return d

move_base_status = ["PENDING", "ACTIVE", "PREEMPTED", "SUCCEEDED", "ABORTED", "REJECTED", "PREEMPTING","RECALLING", "RECALLED", "LOST"]

class AreaGoalMoveBase:
  def __init__(self, robot_frame, world_frame):

    self.marker_pub = rospy.Publisher("area_goal_move_base/goal_area", Marker, queue_size = 1)
    self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)

    self.listener = tf.TransformListener()

    print("wait server")
    self.client.wait_for_server()
    print("finish wait server")

    self.robot_frame = robot_frame
    self.world_frame = world_frame

    self.status = 0
    self.move_base_state = 0
    self.odom = None

    self.area_radius = 0.5

    self.goal = None

    self.timer = 0
    self.time_duration = 0.1
    self.check_move_base_state_time = 3

    rospy.Timer(rospy.Duration(self.time_duration), self.time_callback)
    
  def time_callback(self, msg):

    #  print("timer callback")

      self.timer = self.timer + self.time_duration

      if self.status == 1:
        if self.goal:
          self.marker_pub.publish(marker(self.goal[0], self.goal[1], self.world_frame, self.area_radius))
      else: 
        if self.goal:
          self.marker_pub.publish(marker(self.goal[0], self.goal[1], self.world_frame, self.area_radius, 1))
        #self.marker_pub.publish(non_marker(self.world_frame))

      odom = []

      try:
        (trans,rot) = self.listener.lookupTransform("/" + self.world_frame, "/" + self.robot_frame, rospy.Time(0))
        x = trans[0]
        y = trans[1]
        odom = [x,y]
        
      except:
        pass

      if self.timer > self.check_move_base_state_time:

        self.move_base_state = self.client.get_state()

        if self.move_base_state is 1:
          self.status = 1  
        else:
          self.status = 0      

      d = None
      try:
        d = get_distance(odom, self.goal)
      except:
        pass

      if d:
        if d < self.area_radius:
          self.stop()

  def run(self, p):

    os.system("rosservice call /move_base/clear_costmaps {}")

    x = p[0]
    y = p[1]

    rad = 0
    q = tf.transformations.quaternion_from_euler(0, 0, rad)
    pose = [( x, y, 0.0),( q[0], q[1], q[2], q[3])]
    goal = goal_pose(pose, self.world_frame)

    self.client.send_goal(goal)
    self.goal = [x, y]

    self.status = 1
    self.timer = 0

  def stop(self):
    print("stop")
    self.client.cancel_all_goals()

  def get_status(self):
    s = self.status
    #print("MoveBase:",move_base_status[ms]) 
    #print("AreaGoalMoveBase:",move_base_status[s]) 
    return s

  def get_move_base_status(self):
    ms = self.move_base_state
    #print("MoveBase:",move_base_status[ms]) 
    #print("AreaGoalMoveBase:",move_base_status[s]) 
    return ms


from std_srvs.srv import Empty
from std_srvs.srv import EmptyResponse

import rospy

area_move_base = None
wp = [[0,0.3],[0,-1],[1,0],[-1,0]]
target = 0

def NextHd(req):
  global target
  target = (target + 1) % 4
  area_move_base.run(wp[target])
  return EmptyResponse()

def BackHd(req):
  global target
  target = (target - 1) % 4
  area_move_base.run(wp[target])
  return EmptyResponse()

def StopHd(req):
  global target
  area_move_base.stop()
  return EmptyResponse()

def RunHd(req):
  global target
  area_move_base.run(wp[target])
  return EmptyResponse()

if __name__ == '__main__':
    rospy.init_node('area_goal_move_base')

    rospy.Service('test/next', Empty, NextHd)
    rospy.Service('test/back', Empty, BackHd)
    rospy.Service('test/stop', Empty, StopHd)
    rospy.Service('test/start', Empty, RunHd)

    area_move_base = AreaGoalMoveBase("base_link2", "map2")

    area_move_base.run(wp[target])
    r = rospy.Rate(10) 
    while not rospy.is_shutdown():
      area_move_base.get_status()
      r.sleep()
