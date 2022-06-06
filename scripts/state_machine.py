#!/usr/bin/env python

import sys
import os

sys.path.append(os.path.abspath("./actions"))
sys.path.append(os.path.abspath("./actions/area_goal_move_base"))

from area_goal_move_base import AreaGoalMoveBase
from straight_action import StraightAction
from virtual_action import VirtualAction

import rospy

from std_srvs.srv import Empty
from std_srvs.srv import EmptyResponse

from std_msgs.msg import Float32MultiArray

import rospy


state_machine_state = ["STOP","START","ONE START","LOOP START","START 2 FILE", "START 2 FILE LOOP"]
motion_state = ["STOP","RUN"]
motion = [] 

x_ = None
y_ = None
z_ = None
mode_ = None
last_ = None

px_ = None
py_ = None
pz_ = None

state_ = 0


def callback(msg):

  global x_,y_,z_,mode_,last_
  global px_,py_,pz_

  x_ = msg.data[0]
  y_ = msg.data[1]
  z_ = msg.data[2]
  mode_ = int(msg.data[3])
  last_ = msg.data[4]

  px_ = msg.data[5]
  py_ = msg.data[6]
  pz_ = msg.data[7]

def start():

  global x_,y_,z_,mode_,last_

  if (x_ is not None):
    wp = [x_,y_]
    p_wp = [px_,py_]
    try:
      motion[mode_].run(wp)
    except:
      motion[mode_].run(p_wp, wp)
  else:
    print("ERROR: cannot find target point")

def StartHd(req):
  global state_ 
  state_ = 1
  start()
  return EmptyResponse()

def OnePointStartHd(req):
  global state_ 
  state_ = 2
  start()
  return EmptyResponse()

def LoopStartHd(req):
  global state_ 
  state_ = 3
  start()
  return EmptyResponse()

file_up_counter = 0

def Start2FileHd(req):
  global state_ 
  global file_up_counter

  file_up_counter = 0
  state_ = 4
  start()
  return EmptyResponse()

def Start2FileLoopHd(req):
  global file_up_counter
  global state_ 

  file_up_counter = 0
  state_ = 5
  start()
  return EmptyResponse()

def StopHd(req):
  global state_

  motion[mode_].stop()
  state_ = 0
  
  return EmptyResponse()

if __name__ == '__main__':


    rospy.init_node('state_machine')

    #area_move_base = AreaGoalMoveBase("base_link2", "map2")
    area_move_base = AreaGoalMoveBase("base_link", "map")
    virtual_action = VirtualAction()
    #straight_action = StraightAction("base_link2", "map2")
    straight_action = StraightAction("base_link", "map")
    # append motion
    motion = [area_move_base, area_move_base] 
    #motion = [virtual_action] 
    #motion = [straight_action] 

    rospy.Subscriber("wp_editor/target_point", Float32MultiArray, callback)

    rospy.Service('state_machine/start', Empty, StartHd)
    rospy.Service('state_machine/one_point_start', Empty, OnePointStartHd)
    rospy.Service('state_machine/loop_start', Empty, LoopStartHd)
    rospy.Service('state_machine/stop', Empty, StopHd)
    rospy.Service('state_machine/start_two_file', Empty, Start2FileHd)
    rospy.Service('state_machine/start_two_file_loop', Empty, Start2FileLoopHd)

    service_name = 'wp_editor/up'
    rospy.wait_for_service(service_name)
    wp_up_service = rospy.ServiceProxy(service_name, Empty)

    service_name = 'wp_editor/file/up'
    rospy.wait_for_service(service_name)
    wp_file_up_service = rospy.ServiceProxy(service_name, Empty)

    service_name = 'wp_editor/file/down'
    rospy.wait_for_service(service_name)
    wp_file_down_service = rospy.ServiceProxy(service_name, Empty)

  #  area_move_base.run(wp[target])
    r = rospy.Rate(10) 
    while not rospy.is_shutdown():
      if (mode_ is not None):
        s = motion[mode_].get_status()
        print("motion state:", motion_state[s])
        print("state machine state:",state_machine_state[state_])
        if state_ == 1:
          if s is 0:
            wp_up_service()
            start()
            if last_ == 1:
              state_ = 0

        if state_ == 2:
          if s is 0:
            state_ = 0

        if state_ == 3:
          if s is 0:
            wp_up_service()
            start()

        if state_ == 4:
          if s is 0:
            wp_up_service()
            start()
            if last_ == 1:
              wp_file_up_service()
              if file_up_counter == 0:
                file_up_counter = file_up_counter + 1
              else:
                wp_file_down_service()
                file_up_counter = 0
                state_ = 0

        if state_ == 5:
          if s is 0:
            wp_up_service()
            start()
            if last_ == 1:
              if file_up_counter == 0:
                wp_file_up_service()
                file_up_counter = file_up_counter + 1
              else:
                wp_file_down_service()
                file_up_counter = 0

      #area_move_base.get_status()
      r.sleep()
