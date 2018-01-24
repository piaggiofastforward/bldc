#! /usr/bin/env python

import rospy
import serial
import argparse
import traceback
import time
from std_msgs.msg import UInt8

BAUD = 115200
TIMEOUT = 0.5  # seconds
PACKET_HEADER = 47
PACKET_FOOTER = '\n'
PACKET_SIZE = 5

class DataHandler():

    def __init__(self):
        rospy.init_node('hall_listener')
        self.pubs = [
            rospy.Publisher('/hall_1', UInt8, queue_size = 0),
            rospy.Publisher('/hall_2', UInt8, queue_size = 0),
            rospy.Publisher('/hall_3', UInt8, queue_size = 0)
        ]

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