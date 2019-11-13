#!/usr/bin/env python
import rospy
import math
import csv
import sys
import time
from sensor_msgs.msg import NavSatFix
import serial
from std_msgs.msg import Float32

# open serial port
ser = serial.Serial()
ser.baudrate = 9600
# this port might be different from boats
ser.port = '/dev/ttyS4'
ser.timeout = 0.1
ser.open()
time.sleep(1)
ser.reset_input_buffer()
# set variables
micron_output = 0.00
micron_str = "-1"


def serdataget():
    '''
    Get the serail data from "ser" and save to "micron_str"
    return:    micron_str
    '''
    global micron_str
    if(ser.in_waiting):
        # readline from serial port
        lmao = ser.readline()
        
        # print(lmao.find(',M,'))
    
        if(lmao.find(',M,') != -1):
            micron_str = lmao[16]+lmao[17]+lmao[18]+lmao[19]+lmao[20]+lmao[21]
            # print to check 
            print('micron_str: ', micron_str)
    return micron_str

def serclose():
    '''
    close the serial stream
    '''
    ser.close()


def data_extraction():
    '''
    Open a rostopic and publish data
    '''
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
