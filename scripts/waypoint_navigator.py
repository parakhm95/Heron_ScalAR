#!/usr/bin/env python
import rospy
import math

import sys
import time

from geometry_msgs.msg import Vector3Stamped
from heron_msgs.msg import Helm
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Float32

pos_cur=NavSatFix()
yaw_cur = 0.00
yaw_des_old = 0.0
course_desired = ([49.9000000007,8.89999999997],
	[49.9000000007+0.0005,8.89999999997],
	[49.9000000007+0.0005,8.89999999997+0.0005],
	[49.9000000007,8.89999999997+0.0005],
	[49.9000000007,8.89999999997])
i = 0
kp=1
wypt_dist_thresh = 2.0
cmd_dt = 0.1
base_thrust = 1.0

def nav_comp(navsat_msg):
    global pos_cur
    pos_cur = navsat_msg
    return

def yaw_callback(Float32_msg):
    global yaw_cur
    yaw_cur = Float32_msg.data
    if(yaw_cur < 0):
    	yaw_cur = yaw_cur + 2 * math.pi
    return


def control_pulisher(event):
    global yaw_cur, i, kp, pos_cur, yaw_des_old, course_desiredi, wypt_dist_thresh, cmd_dt, base_thrust
    pub_msg = Helm()
    helm_pub = rospy('/cmd_helm', Helm, queue_size=100)

    pos_des_lat = course_desired[i][0]
    pos_des_lon = course_desired[i][1]

    dist_err = math.sqrt( (pos_des_lat-pos_cur.latitude)**2 + (pos_des_lon - pos_cur.longitude)**2 )
    if( dist_err < wypt_dist_thresh) 
        if( i < len(course_desired)-1 ):
            i = i+1
            pos_des_lat = course_desired[i][0]
            pos_des_lon = course_desired[i][1]
            dist_err = math.sqrt( (pos_des_lat-pos_cur.latitude)**2 + (pos_des_lon - pos_cur.longitude)**2 )
        else:
            pub_msg.thrust = 0.0
            pub_msg.yaw_rate = 0.0
            helm_pub.publish(pub_msg)
            return
    
    yaw_des = math.atan2( pos_des_lat-pos_cur.latitude, pos_des_lon-pos_cur.longitude)
    if( yaw_des < 0):
        yaw_des = 2*math.pi + yaw_des

    yaw_des_rate = (yaw_des-yaw_des_old)/cmd_dt;
    yaw_des_old = yaw_des;

    yaw_error = yaw_des - yaw_cur
    if ( yaw_error > math.pi ):
        yaw_error = yaw_error-2*math.pi
    elif ( yaw_error < -math.pi ):
        yaw_error = 2*math.pi + yaw_error

    pub_msg.yaw_rate = kp*yaw_error + yaw_des_rate
    if( dist_error > 10 ):
        pub_msg.thrust = base_thrust
    else:
        pub_msg.thrust = base_thrust*dist_error/10.0

    pub_msg.thrust = pub_msg.thrust*math.exp(-10*math.fabs(yaw_error) )
    helm_pub.publish(pub_msg)
    return
    
def course_publisher():
    global cmd_dt

    # In ROS, nodes are uniquely named. If two nodes with the same
    # node are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('wapt_control_publisher', anonymous=True)
    rospy.Subscriber("/navsat/fix", NavSatFix,nav_comp)
    
    # /filtered_yaw is the topic on which the data from the complementary filter will arrive
    rospy.Subscriber("/filtered_yaw", Float32, yaw_callback)
    rospy.Timer(rospy.Duration(cmd_dt), control_publisher)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    course_publisher()
