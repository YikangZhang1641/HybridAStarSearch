from costmap_converter.msg import ObstacleArrayMsg
from costmap_converter.msg import ObstacleMsg
from geometry_msgs.msg import Point32
import rospy

def pt(x, y):
  p = Point32()
  p.x = x
  p.y = y
  p.z = 0
  return p

pub = rospy.Publisher("/obstacles", ObstacleArrayMsg)
rospy.init_node("talker")

rate = rospy.Rate(1)
msg = ObstacleArrayMsg()

ob1 = ObstacleMsg()
ob1.header.frame_id = "map"
ob1.polygon.points = [pt(3,5), pt(4,6), pt(5,6), pt(5,2)]

ob2 = ObstacleMsg()
ob2.header.frame_id = "map"
ob2.polygon.points = [pt(7,1), pt(8,5), pt(8,1)]


msg.header.frame_id = "map"
msg.obstacles = [ob1, ob2]


while not rospy.is_shutdown():
  msg.header.stamp = rospy.Time.now()
  pub.publish(msg)
  rate.sleep()