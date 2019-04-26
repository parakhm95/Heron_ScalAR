import rospy
import math
<<<<<<< HEAD
import csv
import sys
import time
from sensor_msgs.msg import NavSatFix
import serial
latitude = []
longitude = []
file = open('data11.csv','a')
data = csv.writer(file, delimiter=',',quotechar='|', quoting=csv.QUOTE_MINIMAL)
ser = serial.Serial()
ser.baudrate = 9600
ser.port = '/dev/ttyUSB0'
ser.timeout = 0.1
ser.open()
ser.reset_input_buffer()
micron = "sounder"


def navsat_get(navsat_msg):
    global latitude, longitude, micron
=======

import sys
import time
from sensor_msgs.msg import NavSatFix
latitude = []
longitude = []


def navsat_get(navsat_msg):
    global latitude, longitude
>>>>>>> master
    latitude.append(navsat_msg.latitude)
    longitude.append(navsat_msg.longitude)
    print navsat_msg.longitude
    print navsat_msg.latitude
<<<<<<< HEAD
    data.writerow([rospy.get_time(),navsat_msg.latitude,navsat_msg.longitude,micron])


def spit():
    global file
    print latitude
    print longitude
    file.close()

def serdataget():
    global micron
        if(ser.in_waiting):
        lmao = ser.readline()
        print(lmao.find(',M,'))
        if(lmao.find(',M,') != -1):
            micron = lmao[16]+lmao[17]+lmao[18]+lmao[19]+lmao[20]+lmao[21]
        # print(lmao[16]+lmao[17]+lmao[18]+lmao[19]+lmao[20]+lmao[21])
    return


=======

def spit():
    print latitude
    print longitude
>>>>>>> master



def data_extraction():
    rospy.init_node('data_extraction', anonymous=True)   
    rospy.Subscriber("/navsat/fix", NavSatFix, navsat_get)
    freq = 100  # hz
    rate = rospy.Rate(freq)
    while not rospy.is_shutdown():
<<<<<<< HEAD
        serdataget()
=======
>>>>>>> master
        rate.sleep()
        # print("running")

    rospy.spin()
    rospy.on_shutdown(spit)	
    



if __name__ == '__main__':
	data_extraction()
