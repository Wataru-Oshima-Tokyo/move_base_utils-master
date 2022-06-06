#!/usr/bin/env python

import rospy
import time

move_base_status = ["PENDING", "ACTIVE", "PREEMPTED", "SUCCEEDED", "ABORTED", "REJECTED", "PREEMPTING","RECALLING", "RECALLED", "LOST"]

class VirtualAction:
  status = 0
  counter = 0
  def __init__(self):
    rospy.Timer(rospy.Duration(1), self.callback)

  def run(self,p):
    print("VirtualAction start:",p)
    self.status = 1

  def stop(self):
    print("VirtualAction stop")
    self.status = 0
    
  def get_status(self):
    s = self.status
    return s

  def callback(self,msg):

    if (self.status == 1):
      self.counter = self.counter + 1
      print("counter:",self.counter)

      if (self.counter > 5):
        self.stop()
        self.counter = 0
    
