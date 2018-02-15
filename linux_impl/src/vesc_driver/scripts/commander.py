#! /usr/bin/env python
'''
Give alternating commands to the VESC:
 - 0A for SECS_PER_CMD seconds
 - 1A for SECS_PER_CMD seconds
'''

import rospy
from vesc_driver.msg import Command

cmd_topic = "/vesc_driver/cmd"
SECS_PER_CMD = 10


class Listener(object):

    def __init__(self):
        rospy.init_node("listener")
        self.pub = rospy.Publisher(cmd_topic, Command, queue_size = 0)


    def __pub(self, val):
        hz = 20
        rate = rospy.Rate(hz)
        for i in range(hz * SECS_PER_CMD):
            self.pub.publish(Command(target_cmd = val))
            rate.sleep()

    def run(self):
        while not rospy.is_shutdown():

            # commands are in MILLIAMPS
            self.__pub(0)
            self.__pub(1000)

if __name__ == '__main__':
    Listener().run()
