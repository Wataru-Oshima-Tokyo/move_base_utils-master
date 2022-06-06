#!/usr/bin/env python

import rospy
import tf
import time
from visualization_msgs.msg import Marker
import math

from pose_util import *
from marker_util import LineMarker2D, CylinderMarker2D

import numpy as np
from geometry_msgs.msg import Twist

move_base_status = ["PENDING", "ACTIVE", "PREEMPTED", "SUCCEEDED", "ABORTED", "REJECTED", "PREEMPTING","RECALLING", "RECALLED", "LOST"]

def rotation_o(u, t, deg=False):

    if deg == True:
        t = np.deg2rad(t)

    R = np.array([[np.cos(t), -np.sin(t)],
                  [np.sin(t),  np.cos(t)]])

    return  np.dot(R, u)

class StraightAction:
  status = 0
  counter = 0
  def __init__(self, robot_frame, world_frame):

    self.area_radius = 0.5
    self.start = None
    self.goal = None
    self.goal_line_p = None
    self.odom = None
    self.quat = None

    self.vel_active = None

    self.robot_frame = robot_frame
    self.world_frame = world_frame

    self.listener = tf.TransformListener()

    self.marker_pub = rospy.Publisher("/straight_action/goal_area", Marker, queue_size = 1)
    self.path_pub = rospy.Publisher('/straight_action/path', Path, queue_size=1)
    self.goal_line_pub = rospy.Publisher('/straight_action/goal_line', Marker, queue_size=1)
    self.vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)

    rospy.Timer(rospy.Duration(0.05), self.callback)

    self.start_point_out_dir = 0
    self.robot_pos_out_dir = 0

  def run(self,p1,p2):
    print("straight action : start")
    self.start = p1
    self.goal = p2
    self.status = 1
    self.generate_goal_line()
    self.get_start_out_vec_dir()

  def stop(self):
    print("straight action : stop")
    self.status = 0

  def generate_goal_line(self):
    goal_line_len = 1
    s = np.array(self.start)
    g = np.array(self.goal)

    v_gs_rot = rotation_o(g - s, np.pi/2)
    u_vec = v_gs_rot / np.linalg.norm(v_gs_rot)
    
    p1 = g - u_vec * goal_line_len/2
    p2 = g + u_vec * goal_line_len/2

    self.goal_line_p = [p1.tolist(),p2.tolist()]

  def get_start_out_vec_dir(self):

    gp1 = self.goal_line_p[0]
    gp2 = self.goal_line_p[1]
    sp = self.start

    v = np.array([gp2[0] - gp1[0], gp2[1] - gp1[1]])
    w = np.array([sp[0] - gp1[0], sp[1] - gp1[1]])

    if np.cross(v, w) >= 0:
      self.start_point_out_dir = 1
    else:
      self.start_point_out_dir = -1

  def get_robot_pos_out_vec_dir(self):
    gp1 = self.goal_line_p[0]
    gp2 = self.goal_line_p[1]
    print("****debug****")
    sp = self.odom
    print(sp)

    v = np.array([gp2[0] - gp1[0], gp2[1] - gp1[1]])
    w = np.array([sp[0] - gp1[0], sp[1] - gp1[1]])

    if np.cross(v, w) >= 0:
      self.robot_pos_out_dir = 1
    else:
      self.robot_pos_out_dir = -1

  def get_status(self):
    s = self.status
    return s

  def pub_path(self):
   
    if self.start and self.goal:

      s = self.start
      g = self.goal

      p1 = ROS2DPoseStamped(s[0], s[1], 0)
      p2 = ROS2DPoseStamped(g[0], g[1], 0)

      path = ROS2DPath([p1,p2], "map2") 
      self.path_pub.publish(path) 

  def calc_vel(self):
    g = self.goal
    s = self.start
    o = self.odom
    q = self.quat

    v1 = np.array([g[0] - s[0], g[1] - s[1]]) 
    v2 = np.array([o[0] - s[0], o[1] - s[1]])

    v1 = v1 / np.linalg.norm(v1)
    v2 = v2 / np.linalg.norm(v2)

    out = np.cross(v1, v2)

    rv = np.array(quaternion_to_2d_vector(Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])
))
    rv = rv / np.linalg.norm(rv)

    out2 = np.cross(v1, rv)

    l = 0.05
    beta1 = -0.2
    beta2 = -0.2

    ang1 = beta1 * out
    ang2 = beta2 * out

    #ang = ang2 
    ang = ang1 + ang2 
    print("angle vel",ang)

    t = Twist()
    t.linear.x = l
    t.angular.z = ang
    return t

  def callback(self,msg):

    if (self.status == 1):
     
      try:
        (trans,rot) = self.listener.lookupTransform("/" + self.world_frame, "/" + self.robot_frame, rospy.Time(0))
        x = trans[0]
        y = trans[1]
        odom = [x,y]
        print(odom)
        self.odom = odom
        self.quat = rot

      except:
        pass

      self.pub_path()
      l_marker = None
      self.get_robot_pos_out_vec_dir()
      if self.robot_pos_out_dir is not self.start_point_out_dir:
        l_marker = LineMarker2D(self.goal_line_p[0],self.goal_line_p[1], frame_id="map2",red=1)
        self.status = 0
      else:
        l_marker = LineMarker2D(self.goal_line_p[0],self.goal_line_p[1], frame_id="map2")

      self.goal_line_pub.publish(l_marker)
      self.vel_pub.publish(self.calc_vel())
      self.vel_active = 1

    else:
      if (self.vel_active):
        self.vel_pub.publish(Twist())
        self.vel_active = 0
            
 
