#! /usr/bin/env python
import serial
import argparse
import struct
import time
import copy
import rospy
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

# start byte, end_byte, 2 crc, packet length, 9 byte payload
PACKET_LENGTH = 14



class CurrentTracker(object):

    def __init__(self):
        self.min = 0.0
        self.max = 0.0

    def update(self, iq):
        self.min = iq if iq < self.min else self.min
        self.max = iq if iq > self.max else self.max

    def unpack(self):
        return {'min': self.min, 'max': self.max}\

comm_steps = [
    [1,0,1],
    [1,0,0],
    [1,1,0],
    [0,1,0],
    [0,1,1],
    [0,0,1],
]


class HallTracker(object):

    def __init__(self):
        self._dict = {
            str(comm_step): CurrentTracker() for comm_step in comm_steps
        }

    def update(self, comm_step, current):
        self._dict[str(comm_step)].update(current)

    def print_results(self):
        for step, c_tracker in self._dict.iteritems():
            print step + str(c_tracker.unpack())

def stringify_list(l):
    ret = ''
    for item in l:
        ret += str(item) + '_'
    return ret[:len(ret) - 1] # leave out last underscore


class Listener(object):

    def __init__(self, port, tracker):
        rospy.init_node('hall_listener')
        self.pubs = {
            str(comm_step): None for comm_step in comm_steps
        }
        get_topic = lambda step: '/comm_' + stringify_list(step)
        for step in comm_steps:
            self.pubs[str(step)] = rospy.Publisher(get_topic(step), UInt8, queue_size = 0)
        self.tracker = tracker
        self.port = port

    # the PACKET_HEADER must be contained within the message so that while
    # know which data point corresponds to which hall sensor
    def publish_all(self, new_data):
        if PACKET_HEADER not in new_data or PACKET_FOOTER not in new_data:
            # print 'packet header and footer not found in the data'
            return -1

        try:
            # packet header, then packet length, then payload
            # payload is 3 hall sensor bytes and 4 motor current bytes
            start = new_data.index(PACKET_HEADER) + 1
            hall_vals = [new_data[index] for index in range(start, start+3)]
            print 'hall sensor values:\n\t' + str(hall_vals)

            # floating point current reading (4 bytes) will be at the end of the packet
            current_bytes = bytearray([new_data[i] for i in range(start + 3, start + 7)])
            current_measured = struct.unpack('f', current_bytes)[0]
            print 'motor current:\n\t%.5f' % current_measured
            self.tracker.update(hall_vals, current_measured)
        except IndexError:
            import pdb; pdb.set_trace()


    def listen(self):
        ser = serial.Serial(self.port, BAUD, timeout=TIMEOUT)

        # continually loop and read the values being sent to us over UART. write to stdout
        f = lambda: ser.read_until(b'\x30')
        while (1):
            data = f()

            # in case there happens to be multiple end bytes within the payload itself
            while len(data) < PACKET_LENGTH:
                data += f()

            #use ord() to convert the bytes to integers
            self.publish_all([ord(d) for d in data])
            # time.sleep(0.2)

    def print_results(self):
        return self.tracker.print_results()



if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--port', type=str, default='/dev/ttyUSB0')
    args = parser.parse_args()
    l = Listener(args.port, HallTracker())

    try:
        l.listen()
    except Exception as e:
        print(e)
        print '\n\n'
        l.print_results()
