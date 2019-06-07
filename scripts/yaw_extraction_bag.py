#!/usr/bin/env python
import rospy
import rosbag
import math
import csv
import tf
import sys
import time
from geometry_msgs.msg import Vector3Stamped
from sensor_msgs.msg import Imu
from tf.transformations import euler_from_quaternion
# ros node begins
rospy.init_node('bag_yaw_extraction', anonymous=True)
euler_deg = [0,0,0]


# csv file opening
file0 = open('yaw_data_bag_01.csv','a')
data0 = csv.writer(file0, delimiter=',',quotechar='|', quoting=csv.QUOTE_MINIMAL)
file1 = open('yaw_rpy_bag_01.csv','a')
data1 = csv.writer(file1, delimiter=',',quotechar='|', quoting=csv.QUOTE_MINIMAL)

# open rosbag
bag = rosbag.Bag('/home/pmg/Documents/Bagfiles_Heron/2019-05-17-20-45-54.bag')

for topic, msg, t in bag.read_messages(topics=['/imu/data', '/imu/rpy']):
    if(topic == '/imu/data'):
        # conversion from quaternion to rpy
        quaternion = (msg.orientation.x,msg.orientation.y,msg.orientation.z,msg.orientation.w)
        euler = tf.transformations.euler_from_quaternion(quaternion)
        euler_deg[0] = (math.degrees(euler[0]))
        euler_deg[1] = (math.degrees(euler[1]))
        euler_deg[2] = (math.degrees(euler[2]))
        print euler_deg
        data0.writerow([euler_deg[0],euler_deg[1],euler_deg[2]])
    elif(topic == '/imu/rpy'):
        euler = [msg.vector.x,msg.vector.y,msg.vector.z]
        euler_deg[0] = (math.degrees(euler[0]))
        euler_deg[1] = (math.degrees(euler[1]))
        euler_deg[2] = (math.degrees(euler[2]))
        print euler_deg 
        data1.writerow([euler_deg[0],euler_deg[1],euler_deg[2]])
bag.close()
file0.close()
file1.close()
