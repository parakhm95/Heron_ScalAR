#!/usr/bin/env python
import rospy
import math
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Vector3Stamped
from std_msgs.msg import Float32
startup = 0
mag_read = None
mag_read_prev = None
yaw = 0.00

def imu_data_callback(msg):
    global startup, mag_read, mag_read_prev, yaw
    if startup == 1:
        yaw_pub = rospy.Publisher('/filtered_yaw', Float32, queue_size=100)
        yaw = yaw - msg.angular_velocity.z * 0.05
        yaw = yaw * 0.996 + mag_read * 0.004
        # print("before1", yaw)
    if yaw > math.pi :
        # print("bigger than pi")
        yaw_out = yaw - (2 * math.pi)
        # print("before2", yaw)
    elif yaw < -math.pi :
        # print("smaller than pi")
        yaw_out = yaw + (2 * math.pi)
    else :
        yaw_out = yaw
        # print("before3", yaw)
    yaw_pub.publish(yaw_out)
    print(math.degrees(mag_read), math.degrees(yaw_out))
    # print("final", yaw)

def imu_mag_callback(msg):
    global startup, mag_read, mag_read_prev, yaw
    if startup == 0 :
        mag_read = math.pi - math.atan2(msg.vector.x,msg.vector.y)
        print("First value is",math.degrees(yaw))
        if (mag_read > math.pi):
            mag_read = mag_read - 2*math.pi
        yaw = mag_read
        mag_read_prev = yaw
        startup = 1
        # print("yaw acquired")
    elif startup == 1 :
        mag_read = math.pi - math.atan2(msg.vector.x,msg.vector.y)
        if (mag_read > math.pi):
            mag_read = mag_read - 2*math.pi

        if (mag_read_prev - mag_read) > math.pi:
            yaw = yaw - 2 * math.pi
            # print("big1")
        elif (mag_read_prev - mag_read) < -math.pi:
            yaw = yaw + 2 * math.pi
            # print("big2")
        # print (mag_read_prev, mag_read)
        # print mag_read_prev-mag_read
        mag_read_prev = mag_read
        # print yaw



def yaw_fusion():
    rospy.init_node('yaw_fusion', anonymous=True)
    rospy.Subscriber('/imu/mag_calib', Vector3Stamped, imu_mag_callback)
    rospy.Subscriber('/imu/data', Imu, imu_data_callback)
    while not rospy.is_shutdown():
        pass
    rospy.spin()

if __name__ == '__main__':
    yaw_fusion()