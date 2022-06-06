#!/usr/bin/env python

"""
Copyright (c) 2011, Willow Garage, Inc.
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

    * Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright
      notice, this list of conditions and the following disclaimer in the
      documentation and/or other materials provided with the distribution.
    * Neither the name of the Willow Garage, Inc. nor the names of its
      contributors may be used to endorse or promote products derived from
      this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
SUBSTITUTE GOODS OR SERVICES LOSS OF USE, DATA, OR PROFITS OR BUSINESS
INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
POSSIBILITY OF SUCH DAMAGE.
"""

import rospy
import copy
import math

from interactive_markers.interactive_marker_server import *
from interactive_markers.menu_handler import *
from visualization_msgs.msg import *
from geometry_msgs.msg import Point

from random import random
from math import sin

from wp_file_manager import WpFileManager

server = None


def processFeedback( feedback ):
    s = "Feedback from marker '" + feedback.marker_name
    s += "' / control '" + feedback.control_name + "'"

    mp = ""
    if feedback.mouse_point_valid:
        mp = " at " + str(feedback.mouse_point.x)
        mp += ", " + str(feedback.mouse_point.y)
        mp += ", " + str(feedback.mouse_point.z)
        mp += " in frame " + feedback.header.frame_id

    if feedback.event_type == InteractiveMarkerFeedback.BUTTON_CLICK:
        rospy.loginfo( s + ": button click" + mp + "." )
    elif feedback.event_type == InteractiveMarkerFeedback.MENU_SELECT:
        rospy.loginfo( s + ": menu item " + str(feedback.menu_entry_id) + " clicked" + mp + "." )
    elif feedback.event_type == InteractiveMarkerFeedback.POSE_UPDATE:
        rospy.loginfo( s + ": pose changed")
# TODO
#          << "\nposition = "
#          << feedback.pose.position.x
#          << ", " << feedback.pose.position.y
#          << ", " << feedback.pose.position.z
#          << "\norientation = "
#          << feedback.pose.orientation.w
#          << ", " << feedback.pose.orientation.x
#          << ", " << feedback.pose.orientation.y
#          << ", " << feedback.pose.orientation.z
#          << "\nframe: " << feedback.header.frame_id
#          << " time: " << feedback.header.stamp.sec << "sec, "
#          << feedback.header.stamp.nsec << " nsec" )
    elif feedback.event_type == InteractiveMarkerFeedback.MOUSE_DOWN:
        rospy.loginfo( s + ": mouse down" + mp + "." )
    elif feedback.event_type == InteractiveMarkerFeedback.MOUSE_UP:
        rospy.loginfo( s + ": mouse up" + mp + "." )
    server.applyChanges()

def alignMarker( feedback ):
    name = feedback.marker_name
    pose = feedback.pose

    #x = round(pose.position.x-0.5)+0.5
    #y = round(pose.position.y-0.5)+0.5
    x = pose.position.x
    y = pose.position.y


    pose.position.x = x
    pose.position.y = y

    rospy.loginfo(name  + ": aligning position = " + str(feedback.pose.position.x) + "," + str(feedback.pose.position.y) + "," + str(feedback.pose.position.z) + " to " +
                                                                     str(pose.position.x) + "," + str(pose.position.y) + "," + str(pose.position.z) )

    server.setPose( feedback.marker_name, pose )
    server.applyChanges()

    global target_id
    ps[int(name)][0] = x
    ps[int(name)][1] = y

def rand( min_, max_ ):
    return min_ + random()*(max_-min_)

def makeBox( msg ):
    marker = Marker()

    marker.type = Marker.SPHERE
    marker.scale.x = msg.scale * 0.3
    marker.scale.y = msg.scale * 0.3
    marker.scale.z = msg.scale * 0.3
    marker.color.r = 0.0
    marker.color.g = 0.5
    marker.color.b = 0.0
    marker.color.a = 1.0

    return marker

def makeBox1( msg ):
    marker = Marker()

    marker.type = Marker.SPHERE
    marker.scale.x = msg.scale * 0.3
    marker.scale.y = msg.scale * 0.3
    marker.scale.z = msg.scale * 0.3
    marker.color.r = 0.5
    marker.color.g = 0.5
    marker.color.b = 0.0
    marker.color.a = 1.0

    return marker

def makeBoxRed( msg ):
    marker = Marker()

    marker.type = Marker.SPHERE
    marker.scale.x = msg.scale * 0.3
    marker.scale.y = msg.scale * 0.3
    marker.scale.z = msg.scale * 0.3
    marker.color.r = 0.5
    marker.color.g = 0.0
    marker.color.b = 0.0
    marker.color.a = 1.0

    return marker

def non_marker(frame):
    marker_data = Marker()
    marker_data.header.frame_id = frame
    marker_data.ns = "basic_shapes"
    marker_data.id = 0
    return marker_data

def cylinder_marker(x,y,z,frame,radius,gray=0):

    marker_data = Marker()
    marker_data.header.frame_id = frame
    marker_data.header.stamp = rospy.Time.now()

    marker_data.ns = "basic_shapes"
    marker_data.id = 0

    marker_data.action = Marker.ADD

    marker_data.pose.position.x = x
    marker_data.pose.position.y = y
    marker_data.pose.position.z = z

    if gray:

      marker_data.color.r = 0.3
      marker_data.color.g = 0.3
      marker_data.color.b = 0.3

    else:

      marker_data.color.r = 0.5
      marker_data.color.g = 0.5
      marker_data.color.b = 0.0


    marker_data.color.a = 0.5
    marker_data.scale.x = radius * 2
    marker_data.scale.y = radius * 2
    marker_data.scale.z = 0.01

    marker_data.lifetime = rospy.Duration(0.2)
    marker_data.type = 3

    return marker_data
#####################################################################
# Marker Creation

def normalizeQuaternion( quaternion_msg ):
    norm = quaternion_msg.x**2 + quaternion_msg.y**2 + quaternion_msg.z**2 + quaternion_msg.w**2
    s = norm**(-0.5)
    quaternion_msg.x *= s
    quaternion_msg.y *= s
    quaternion_msg.z *= s
    quaternion_msg.w *= s

def makeChessPieceMarker(position, id, frame, target=0, mode=0):
    int_marker = InteractiveMarker()
    int_marker.header.frame_id = frame
    int_marker.pose.position = position
    int_marker.scale = 1

    int_marker.name = str(id)
    int_marker.description = "ID:" + str(id) + "\n Mode:" + str(mode)

    control = InteractiveMarkerControl()
    control.orientation.w = 1
    control.orientation.x = 0
    control.orientation.y = 1
    control.orientation.z = 0
    normalizeQuaternion(control.orientation)
    control.interaction_mode = InteractiveMarkerControl.MOVE_PLANE
    int_marker.controls.append(copy.deepcopy(control))

    # make a box which also moves in the plane
    if mode==0:
      control.markers.append( makeBox(int_marker) )
    if mode==1:
      control.markers.append( makeBox1(int_marker) )

    control.always_visible = True
    int_marker.controls.append(control)

    # we want to use our special callback function
    server.insert(int_marker, processFeedback)

    # set different callback for POSE_UPDATE feedback
    server.setCallback(int_marker.name, alignMarker, InteractiveMarkerFeedback.POSE_UPDATE )

#p(x,y,z,mode)
ps = []
target_id = 0


for i in range(5):
  p = [i + 0.5 ,0.5, 0.0, 0]
  ps.append(p)

def set_points(ps):
  server.clear()

  for i in range(len(ps)):
    position = Point( ps[i][0], ps[i][1], ps[i][2])
    if i == target_id:
      makeChessPieceMarker(position, i, world_frame_, 1, ps[i][3])
    else:
      makeChessPieceMarker(position, i, world_frame_, 0, ps[i][3])

    server.applyChanges()

def add_wp_handle(req):
  global target_id
  print("add wp")
  p = ps[target_id]
  tid = target_id
  ang = 0

  dis = 2.0

  if len(ps)>1:
    p1 = ps[tid - 1]
    y1 = p1[1]
    x1 = p1[0]

    p2 = ps[tid]
    y2 = p2[1]
    x2 = p2[0]

    ang = math.atan2( y2 - y1 , x2 - x1 )

  ps.insert(target_id + 1, [p[0] + dis * math.cos(ang) ,p[1] + dis * math.sin(ang), p[2], p[3]])
  target_id = target_id + 1
  set_points(ps)
  return EmptyResponse()

def del_wp_handle(req):
  global target_id

  del_id = target_id
  
  if (len(ps)>1):

    if (target_id == len(ps) - 1):
      target_id = target_id - 1

    ps.pop(del_id)

  set_points(ps)
  return EmptyResponse()

def clear_wp_handle(req):
  global target_id
  global ps

  p = ps[target_id]

  ps = [p]

  target_id = 0
  
  set_points(ps)

def up_wp_handle(req):
 
  print("up wp")

  global target_id

  if target_id == len(ps) - 1:
    target_id = 0
  else:
    target_id = target_id + 1

  set_points(ps)
  return EmptyResponse()

def get_pre_target_id():
  global target_id

  p_id = None
 
  if target_id == 0:
    p_id = len(ps) - 1
  else:
    p_id = target_id - 1
  
  return p_id

def down_wp_handle(req):

  global target_id

  print("down wp")

  target_id = get_pre_target_id()

  set_points(ps)
  return EmptyResponse()

def get_target_pos_handle(req):
  global target_id
  print(ps[target_id])
  return EmptyResponse()

def mode_change_handle(req):
  global target_id
  ps[target_id][3] = 1
  set_points(ps)
  return EmptyResponse()

def reverse_wp_handle(req):
  global ps

  ps.reverse()
  set_points(ps)

  return EmptyResponse()

def nav_start_handle(req):
  pass

def nav_stop_handle(req):
  pass

def nav_start_one_p_handle(req):
  pass

def nav_start_loop_handle(req):
  pass

def file_save_handle(req):
  global wf_id
  global wm
  wm.write_wp(wf_id, ps)
  return EmptyResponse()

def file_copy_handle(req):
  global wf_id
  global max_wp_file_size
  global wm

  print("copy handle")

  save_id = wf_id + 1
  if wf_id == max_wp_file_size:
    save_id = 1

  wm.write_wp(save_id, ps)

  return EmptyResponse()

def file_up_handle(req):
  global wf_id
  global wm
  global max_wp_file_size
  global target_id
  global ps

  target_id = 0

  if wf_id == max_wp_file_size:
    wf_id = 1
  else:
    wf_id = wf_id + 1

  wp = wm.get_wp(wf_id)
  ps = wp

  set_points(ps)
  return EmptyResponse()
  

def file_down_handle(req):
  global wf_id
  global wm
  global max_wp_file_size
  global target_id
  global ps

  target_id = 0

  if wf_id == 1:
    wf_id = max_wp_file_size
  else:
    wf_id = wf_id - 1

  wp = wm.get_wp(wf_id)
  ps = wp

  set_points(ps)
  return EmptyResponse()

from std_srvs.srv import Empty
from std_srvs.srv import EmptyResponse
import rospy

from std_msgs.msg import Float32MultiArray

world_frame_ = "map2"

wm = None
wf_id = None

max_wp_file_size = 10

if __name__=="__main__":

    rospy.init_node("int_wp_editor")

    server = InteractiveMarkerServer("basic_controls")
    waypoints_pub = rospy.Publisher('wp_editor/target_point', Float32MultiArray, queue_size=1) 
    target_marker_pub = rospy.Publisher('wp_editor/target_point_marker', Marker, queue_size=1) 

    rospy.Service("wp_editor/add", Empty, add_wp_handle)
    rospy.Service("wp_editor/del", Empty, del_wp_handle)
    rospy.Service("wp_editor/clear", Empty, clear_wp_handle)
    rospy.Service("wp_editor/up", Empty, up_wp_handle)
    rospy.Service("wp_editor/down", Empty, down_wp_handle)
    rospy.Service("wp_editor/mode_change", Empty, mode_change_handle)
    rospy.Service("wp_editor/get_target_pos", Empty, get_target_pos_handle)
    rospy.Service("wp_editor/reverse", Empty, reverse_wp_handle)

    rospy.Service("wp_editor/file/save", Empty, file_save_handle)
    rospy.Service("wp_editor/file/new", Empty, up_wp_handle)
    rospy.Service("wp_editor/file/del", Empty, up_wp_handle)
    rospy.Service("wp_editor/file/up", Empty, file_up_handle)
    rospy.Service("wp_editor/file/down", Empty, file_down_handle)
    rospy.Service("wp_editor/file/copy", Empty, file_copy_handle)

    wm = WpFileManager("wp")
    wf_id = wm.get_last_id()
    wp = wm.get_wp(wf_id)

    ps = wp 

    set_points(ps)

    r = rospy.Rate(10) 
    while not rospy.is_shutdown():
      print("loop")

      tp = ps[target_id]
      pre_tp = ps[get_pre_target_id()]

      print("target point:",tp)
      p = Float32MultiArray()

      x = tp[0]
      y = tp[1]
      z = tp[2]

      px = pre_tp[0]
      py = pre_tp[1]
      pz = pre_tp[2]

      mode = tp[3]

      last = 0
      if (len(ps) - 1) == target_id:
        last = 1

      p.data.append(x)
      p.data.append(y)
      p.data.append(z)
      p.data.append(mode)
      p.data.append(last)
      p.data.append(px)
      p.data.append(py)
      p.data.append(pz)

      print("pub msg:",p)

      waypoints_pub.publish(p)

      cm = cylinder_marker(x, y, z, world_frame_, 0.3)
      target_marker_pub.publish(cm)

      r.sleep() 
