#!/usr/bin/env python
import rospy
import math

import sys
import time

from geometry_msgs.msg import Vector3Stamped
from heron_msgs.msg import Course
from sensor_msgs.msg import NavSatFix

publ=Course()
pos_des=NavSatFix()
pos_cur=NavSatFix()
yaw_cur = 0.00
yaw_des = 0.00
course_desired = ([49.9000000007,8.89999999997],
	[49.9000000007+0.0005,8.89999999997],
	[49.9000000007+0.0005,8.89999999997+0.0005],
	[49.9000000007,8.89999999997+0.0005],
	[49.9000000007,8.89999999997])
i = 0
# yaw_cur=Vector3Stamped()
# yaw_des=Vector3Stamped()
# pos_des.latitude=49.9000000007
# pos_des.longitude=8.89999999997
pos_kp=30000

def nav_comp(navsat_msg):
	global pos_des, pos_kp, publ, yaw_cur, pos_cur, yaw_des,course_desired, i
	pos_cur = navsat_msg
	pos_des.latitude = course_desired[i][0]
	pos_des.longitude = course_desired[i][1]
	yaw_des = math.atan2(pos_des.latitude - pos_cur.latitude, pos_des.longitude - pos_cur.longitude)
	if (yaw_des < 0):
		yaw_des = yaw_des + 2 * math.pi
	distance = math.sqrt((navsat_msg.latitude - pos_des.latitude)**2 + (navsat_msg.longitude - pos_des.longitude)**2)
	yaw_diff = yaw_des - yaw_cur
	if(yaw_diff > math.pi):
		yaw_diff = 2 * math.pi - yaw_diff
	publ.speed = pos_kp * distance * math.exp(-30*(yaw_diff))
	# print(math.exp(-3*(yaw_des - yaw_cur)))
	if(publ.speed > 1.4):
		publ.speed = 1.4
	publ.yaw = yaw_des
	course_publ=rospy.Publisher('/cmd_course', Course, queue_size=100)
	if(distance < 0.0001 and i == len(course_desired) - 1):
		publ.yaw = 0
		publ.speed = 0
		course_publ.publish(publ)
	else:
		course_publ.publish(publ)
	print(publ)
	print('distance away {0}'.format(distance))
	print('going to {0} {1} which is {2} point'.format(pos_des.latitude,pos_des.longitude,i+1))
	if(distance < 0.0001 and (i < len(course_desired) - 1)):
		i = i + 1

	# print(yaw_cur)
	# print(yaw_des)

def yaw_comp(vector3Stamped_msg):
	global yaw_cur
	yaw_cur = vector3Stamped_msg.vector.z
	if(yaw_cur < 0):
		yaw_cur = yaw_cur + 2 * math.pi
	pass


def course_publisher():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # node are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('course_publisher', anonymous=True)
    rospy.Subscriber("/navsat/fix", NavSatFix,nav_comp)
    rospy.Subscriber("/imu/rpy", Vector3Stamped,yaw_comp)
    
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    course_publisher()
