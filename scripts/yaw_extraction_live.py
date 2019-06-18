#!/usr/bin/env python
import rospy
import math
import csv
import tf
import sys
import time
from geometry_msgs.msg import Vector3Stamped
from std_msgs.msg import Float32
from sensor_msgs.msg import Imu
from tf.transformations import euler_from_quaternion
euler_deg = [0,0,0]
euler_rpy = [0,0,0]
euler_compass = [0,0,0]
euler_heading = 0.00
euler_raw = 0.00

# csv file opening
file0 = open('yaw_data_02.csv','a')
data0 = csv.writer(file0, delimiter=',',quotechar='|', quoting=csv.QUOTE_MINIMAL)
file1 = open('yaw_rpy_02.csv','a')
data1 = csv.writer(file1, delimiter=',',quotechar='|', quoting=csv.QUOTE_MINIMAL)

def Imu_data(Imu_msg):
    global euler_deg
    quaternion = (Imu_msg.orientation.x,Imu_msg.orientation.y,Imu_msg.orientation.z,Imu_msg.orientation.w)
    euler = tf.transformations.euler_from_quaternion(quaternion)
    euler_deg[0] = (math.degrees(euler[0]))
    euler_deg[1] = (math.degrees(euler[1]))
    euler_deg[2] = (math.degrees(euler[2]))
    # rospy.loginfo("data : %s",euler_deg[2])

def Imu_compass(Imu_msg):
    global euler_compass
    quaternion = (Imu_msg.orientation.x,Imu_msg.orientation.y,Imu_msg.orientation.z,Imu_msg.orientation.w)
    euler = tf.transformations.euler_from_quaternion(quaternion)
    euler_compass[0] = (math.degrees(euler[0]))
    euler_compass[1] = (math.degrees(euler[1]))
    euler_compass[2] = (math.degrees(euler[2]))
    # rospy.loginfo("compass : %s",euler_compass[2])
    # data0.writerow([euler_deg[0],euler_deg[1],euler_deg[2]])

def Imu_rpy(Imu_msg):
    global euler_rpy
    euler_rpy[0] = (math.degrees(Imu_msg.vector.x))
    euler_rpy[1] = (math.degrees(Imu_msg.vector.y))
    euler_rpy[2] = (math.degrees(Imu_msg.vector.z))

def Imu_compass_heading(Imu_msg):
    global euler_heading
    euler_heading = math.degrees(Imu_msg.data)


def Imu_raw_compass_heading(Imu_msg):
    global euler_raw
    euler_raw = math.degrees(Imu_msg.data)
    # rospy.loginfo("rpy : %s",euler_deg[2])
    # print euler_rpy
    # data1.writerow([euler_rpy[0],euler_rpy[1],euler_rpy[2]])


def spit():
    file0.close()
    file1.close()

# def serdataget():
#     global micron
#     if(ser.in_waiting):
#         lmao = ser.readline()
#         print(lmao.find(',M,'))
#         if(lmao.find(',M,') != -1):
#             micron = lmao[16]+lmao[17]+lmao[18]+lmao[19]+lmao[20]+lmao[21]
#         # print(lmao[16]+lmao[17]+lmao[18]+lmao[19]+lmao[20]+lmao[21])
#     return





def yaw_extraction():
    rospy.init_node('yaw_extraction', anonymous=True)   
    rospy.Subscriber("/imu/data_compass", Imu, Imu_compass)
    rospy.Subscriber("/imu/data", Imu, Imu_data)
    rospy.Subscriber("/imu/rpy", Vector3Stamped, Imu_rpy)
    rospy.Subscriber("/imu/compass_heading", Float32, Imu_compass_heading)
    rospy.Subscriber("/imu/raw_compass_heading", Float32, Imu_raw_compass_heading)
    freq = 100  # hz
    rate = rospy.Rate(freq)
    while not rospy.is_shutdown():
        rospy.loginfo("data : %s, compass : %s, compass_heading : %s, raw_compass_heading : %s",euler_deg[2],euler_compass[2],euler_heading, euler_raw)
        # serdataget()
        rate.sleep()
        # print("running")

    rospy.spin()
    rospy.on_shutdown(spit)	
    



if __name__ == '__main__':
	yaw_extraction()
