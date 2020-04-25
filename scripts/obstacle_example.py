#!/usr/bin/env python  
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

pub = rospy.Publisher("/obstacles", ObstacleArrayMsg, queue_size=1)
rospy.init_node("talker")

rate = rospy.Rate(10)
msg = ObstacleArrayMsg()

ob1 = ObstacleMsg()
ob1.header.frame_id = "map"
# ob1.polygon.points = [pt(-5.1, 4.9), pt(4.9,4.9), pt(4.9, -5.1), pt(5.1,-5.1), pt(5.1,5.1), pt(-5.1,5.1)]
ob1.polygon.points = [pt(-5,5), pt(-4,6), pt(-3,6), pt(-3,2)]

ob2 = ObstacleMsg()
ob2.header.frame_id = "map"
# ob2.polygon.points = [pt(-5.1,-5.1), pt(-4.9, -5.1), pt(-4.9, 4.9), pt(-5.1, 4.9)]
ob2.polygon.points = [pt(5,1), pt(2, 6), pt(4, 7), pt(6,5), pt(6,1)]

ob3 = ObstacleMsg()
ob3.header.frame_id = "map"
# ob3.polygon.points = [pt(0,5), pt(0,10), pt(2,10), pt(2, 5)]
ob3.polygon.points = [pt(-4, -3), pt(0, -5), pt(3, -4), pt(5, -1)]

msg.header.frame_id = "map"
msg.obstacles = [ob1, ob2, ob3]

# spliter = ObstacleMsg()
# spliter.header.frame_id = "map"
# spliter.polygon.points = []
# spliter.polygon.points = [pt(1, -10), pt(1,10), pt(-1, 10), pt(-1, -10)]
# msg.obstacles = [spliter]

while not rospy.is_shutdown():
  msg.header.stamp = rospy.Time.now()
  pub.publish(msg)
  rate.sleep()