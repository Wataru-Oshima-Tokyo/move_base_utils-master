from area_goal_move_base import AreaGoalMoveBase
import rospy

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
