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
kp=2
wypt_dist_thresh = 2.0  # distance threshold to stop or switch to next way point
cmd_dt = 0.1            # time interval for cmd_helm publishing
base_thrust = 1.0       # the base thrust level at which the Heron will run

# conversions from lon/lat to meters
met_lat = 111033.4717 
met_lon = 85467.2528

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


def control_publisher(event):
    global yaw_cur, i, kp, pos_cur, yaw_des_old, course_desired, wypt_dist_thresh, cmd_dt, base_thrust, met_lat, met_lon
    pub_msg = Helm()
    helm_pub = rospy.Publisher('/cmd_helm', Helm, queue_size=100)

    pos_des_lat = course_desired[i][0]
    pos_des_lon = course_desired[i][1]

    delta_lat = (pos_des_lat-pos_cur.latitude)*met_lat
    delta_lon = (pos_des_lon - pos_cur.longitude)*met_lon
    dist_err = math.sqrt( delta_lat**2 + delta_lon**2 )
    
    if (dist_err < wypt_dist_thresh) :
        # go to next way point if within distance threshold
        if( i < len(course_desired)-1 ):
            i = i+1
            pos_des_lat = course_desired[i][0]
            pos_des_lon = course_desired[i][1]
            delta_lat = (pos_des_lat-pos_cur.latitude)*met_lat
            delta_lon = (pos_des_lon - pos_cur.longitude)*met_lon
            dist_err = math.sqrt( delta_lat**2 + delta_lon**2 )
        # stop if at last waypoint
        else:
            pub_msg.thrust = 0.0
            pub_msg.yaw_rate = 0.0
            helm_pub.publish(pub_msg)
            return
    
    yaw_des = math.atan2( delta_lat, delta_lon)
    if( yaw_des < 0):
        yaw_des = 2*math.pi + yaw_des

    # compute rate of change of yaw_des
    yaw_des_rate = (yaw_des-yaw_des_old)/cmd_dt;
    yaw_des_old = yaw_des;

    # compute yaw_error and change it to be within [-pi pi]
    yaw_error = yaw_des - yaw_cur
    if ( yaw_error > math.pi ):
        yaw_error = yaw_error-2*math.pi
    elif ( yaw_error < -math.pi ):
        yaw_error = 2*math.pi + yaw_error

    # controller for yaw_rate
    pub_msg.yaw_rate = kp*yaw_error + yaw_des_rate
    
    # if far away, thrusht is at base thrust level
    if( dist_err > 10 ):
        pub_msg.thrust = base_thrust    
    # gradually slow down as wel approach the waypoint
    else:
        pub_msg.thrust = base_thrust*dist_err/10.0

    # adjust thrust according to yaw_error, i.e., adjust yaw first before thrusting forwayd
    pub_msg.thrust = pub_msg.thrust*math.exp(-10*math.fabs(yaw_error) )
    
    # publish message on topic
    helm_pub.publish(pub_msg)
    
    # print("yaw desired = ", yaw_des)
    # print(" yaw current = ", yaw_cur)
    # print("yaw error= ", yaw_error)
    # print("yaw rate desired= ", pub_msg.yaw_rate)
    return
    
def course_publisher():
    global cmd_dt

    rospy.init_node('wapt_control_publisher', anonymous=True)
    rospy.Subscriber("/navsat/fix", NavSatFix,nav_comp)
    
    # /filtered_yaw is the topic on which the data from the complementary filter will arrive
    rospy.Subscriber("/filtered_yaw", Float32, yaw_callback)
    
    # publishes data on cmd_helm every cmd_dt seconds
    rospy.Timer(rospy.Duration(cmd_dt), control_publisher)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    course_publisher()
