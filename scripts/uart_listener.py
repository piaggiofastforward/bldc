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

    # the PACKET_HEADER must be contained within the message so that we
    # know which data point corresponds to which hall sensor
    def publish_all(self, new_data):
        if PACKET_HEADER not in new_data:
            print 'packet header not found in the data'
            return -1

        start = new_data.index(PACKET_HEADER)
        for sensor_num in [0,1,2]:
            val = UInt8(new_data[sensor_num + start + 1])
            self.pubs[sensor_num].publish(val)


def listen(port):
    data_handler = DataHandler()
    ser = serial.Serial(args.port, BAUD, timeout=TIMEOUT)

    try:
        ser.open()
    except serial.SerialException:
        traceback.print_exc()
        return

    # continually loop and read the values being sent to us over UART. Publish them to 
    bytesToRead = ser.inWaiting()
    while not rospy.is_shutdown():
        while bytesToRead < PACKET_SIZE:
            bytesToRead = ser.inWaiting()
        data = ser.read_until(PACKET_FOOTER, bytesToRead)
        data_handler.publish_all(data)
        time.sleep(0.005)



if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--port', type=str, default='/dev/ttyUSB0')
    args = parser.parse_args()

    try:
        listen(args.port)
    except rospy.ROSInterruptException:
        print 'exiting'