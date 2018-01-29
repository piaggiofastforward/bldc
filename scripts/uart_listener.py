#! /usr/bin/env python

import rospy
import serial
import argparse
import traceback
import time
from std_msgs.msg import UInt8

BAUD = 115200
TIMEOUT = 0.5  # seconds

'''
since we aren't using the packet interface that the VESC firmware provides
(cause its in C and python is much better for rapid development),
use these packet headers and footers on the VESC side.

This won't conflict with any other values since the hall data sent from the
VESC will either be a 0 or a 1
'''

PACKET_HEADER = 47
PACKET_FOOTER = 48
PACKET_SIZE = 5

NUM_HALLS = 3

class DataHandler():

    def __init__(self):
        rospy.init_node('hall_listener')
        self.pubs = []
        get_topic = lambda num: '/hall_' + str(num + 1)
        for i in range(NUM_HALLS):
            self.pubs.append(
                rospy.Publisher(get_topic(i), UInt8, queue_size = 0)
            )

    # the PACKET_HEADER must be contained within the message so that while
    # know which data point corresponds to which hall sensor
    def publish_all(self, new_data):

        display_str = ''
        for x in new_data:
            display_str = display_str + str(x) + ', '
        print display_str

        if PACKET_HEADER not in new_data and PACKET_FOOTER not in new_data:
            print 'packet header and footer not found in the data'
            return -1

        end = new_data.index(PACKET_FOOTER)
        for sensor_num in [0,1,2]:
            val = UInt8(new_data[end - NUM_HALLS + sensor_num])
            self.pubs[sensor_num].publish(val)


def listen(port):
    data_handler = DataHandler()
    ser = serial.Serial(args.port, BAUD, timeout=TIMEOUT)

    # continually loop and read the values being sent to us over UART. Publish them to 
    bytesToRead = ser.inWaiting()
    while not rospy.is_shutdown():
        while bytesToRead < PACKET_SIZE:        
            bytesToRead = ser.inWaiting()
        data = ser.read_until(b'\x30')

        #use ord() to convert the bytes to integers
        data_handler.publish_all([ord(d) for d in data])



if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--port', type=str, default='/dev/ttyUSB0')
    args = parser.parse_args()

    try:
        listen(args.port)
    except rospy.ROSInterruptException:
        print 'exiting'