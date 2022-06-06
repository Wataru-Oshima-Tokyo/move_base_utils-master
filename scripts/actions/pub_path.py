# import for ros function
import rospy
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Header
from nav_msgs.msg import Path

def angle2quat():
  pass

rospy.init_node('path_pub')

r = rospy.Rate(50)
path_pub = rospy.Publisher("/path", Path, queue_size=1)

path_header = Header()
path_header.frame_id = "map2"

path = Path()
path.header = path_header

while not rospy.is_shutdown():


