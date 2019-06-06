#!/usr/bin/env python
import rospy
import math
import csv
import sys
import time
from sensor_msgs.msg import NavSatFix
import serial
from std_msgs.msg import Float32

ser = serial.Serial()
ser.baudrate = 9600
ser.port = '/dev/ttyUSB0'
ser.timeout = 0.1
ser.open()
time.sleep(1)
ser.reset_input_buffer()
micron_output=0.00
micron_str = "0.001"


def serdataget():
    global micron_str
    if(ser.in_waiting):
        lmao = ser.readline()
        print(lmao.find(',M,'))
        print(lmao)
        if(lmao.find(',M,') != -1):
            micron_str = lmao[16]+lmao[17]+lmao[18]+lmao[19]+lmao[20]+lmao[21]
    return micron_str

def serclose():
    ser.close()


def data_extraction():
    global micron_output
    rospy.init_node('micron_echosounder', anonymous=True)
    pub = rospy.Publisher('/echosounder', Float32, queue_size=5)
    freq=100  # hz
    rate=rospy.Rate(freq)
    while not rospy.is_shutdown():
        micron_output = float(serdataget())
        pub.publish(micron_output)
        rate.sleep()
        # print("running")

    rospy.spin()
    rospy.on_shutdown(serclose)


if __name__ == '__main__':
    try:
        data_extraction()
    except rospy.ROSInterruptException:
        pass
