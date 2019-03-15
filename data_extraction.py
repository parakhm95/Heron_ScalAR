import rospy
import math

import sys
import time
from sensor_msgs.msg import NavSatFix
latitude = []
longitude = []


def navsat_get(navsat_msg):
    global latitude, longitude
    latitude.append(navsat_msg.latitude)
    longitude.append(navsat_msg.longitude)
    print navsat_msg.longitude
    print navsat_msg.latitude

def spit():
    print latitude
    print longitude



def data_extraction():
    rospy.init_node('data_extraction', anonymous=True)   
    rospy.Subscriber("/navsat/fix", NavSatFix, navsat_get)
    freq = 100  # hz
    rate = rospy.Rate(freq)
    while not rospy.is_shutdown():
        rate.sleep()
        # print("running")

    rospy.spin()
    rospy.on_shutdown(spit)	
    



if __name__ == '__main__':
	data_extraction()
