from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
import rospy

def CylinderMarker2D(p,radius,z=0,frame_id="map",red=0):

    marker = Marker()
    marker.header.frame_id = frame_id

    marker.ns = "basic_shapes"
    marker.id = 0

    marker.action = Marker.ADD

    marker.pose.position.x = p[0]
    marker.pose.position.y = p[1]
    marker.pose.position.z = z

    marker.pose.orientation.x = 0.0
    marker.pose.orientation.y = 0.0
    marker.pose.orientation.z = 0.0
    marker.pose.orientation.w = 1.0

    if red:

      marker.color.r = 0.5
      marker.color.g = 0
      marker.color.b = 0

    else:

      marker.color.r = 0.5
      marker.color.g = 0.5
      marker.color.b = 0.0


    marker.color.a = 0.5
    marker.scale.x = radius * 2
    marker.scale.y = radius * 2
    marker.scale.z = 0.01

    marker.lifetime = rospy.Duration(0.2)
    marker.type = 3

    return marker

def LineMarker2D(p1,p2,z=0,frame_id="map",red=0):
    marker = Marker()
    marker.header.frame_id = frame_id
    marker.type = marker.LINE_STRIP
    marker.action = marker.ADD

    # marker scale
    marker.scale.x = 0.1
    marker.scale.y = 0.1
    marker.scale.z = 0.1

    # marker color
    if red:
      marker.color.a = 1.0
      marker.color.r = 1.0
      marker.color.g = 0.0
      marker.color.b = 0.0

    else:
      marker.color.a = 1.0
      marker.color.r = 1.0
      marker.color.g = 1.0
      marker.color.b = 0.0

    # marker orientaiton
    marker.pose.orientation.x = 0.0
    marker.pose.orientation.y = 0.0
    marker.pose.orientation.z = 0.0
    marker.pose.orientation.w = 1.0

    # marker position
    marker.pose.position.x = 0.0
    marker.pose.position.y = 0.0
    marker.pose.position.z = 0.0

    # marker line points
    marker.points = []
    # first point
    first_line_point = Point()
    first_line_point.x = p1[0]
    first_line_point.y = p1[1]
    first_line_point.z = z
    marker.points.append(first_line_point)
    # second point
    second_line_point = Point()
    second_line_point.x = p2[0]
    second_line_point.y = p2[1]
    second_line_point.z = z
    marker.lifetime = rospy.Duration(0.2)
    marker.points.append(second_line_point)

    return marker


