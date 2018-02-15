#! /usr/bin/env python

import rospy
from vesc_driver.msg import Feedback

rmotor_fb_topic = "/r5/rmotor/feedback"


class Listener(object):

    def __init__(self):
        rospy.init_node("listener")
        self.r_fb_sub = rospy.Subscriber(
            rmotor_fb_topic, Feedback, self.rmotor_fb_cb
        )

        # remember if the count has reset from -1 to -360
        self.last_measured_pos = 0
        self.got_first_cb = False
        self.rotations = 0


    def rmotor_fb_cb(self, msg):

        # this condition means the count has overflowed
        if self.got_first_cb:
            if msg.measured_position < self.last_measured_pos - 45:
                self.rotations += 1
                print "rotations: " + str(self.rotations)
        else:
            self.got_first_cb = True
        self.last_measured_pos = msg.measured_position

        # print "measured position: " + str(msg.measured_position)


    def run(self):
        rate = rospy.Rate(100)

        while not rospy.is_shutdown():
            rate.sleep()

if __name__ == '__main__':
    Listener().run()
