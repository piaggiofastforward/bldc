#! /usr/bin/env python

import rospy
from vesc_driver.msg import Feedback

linkage_fb_topic = "/linkage/feedback"

feedback_file = '/home/benb/Documents/feedback_results.csv'

class Listener(object):

    def __init__(self):
        rospy.init_node("listener")
        self.r_fb_sub = rospy.Subscriber(
            linkage_fb_topic, Feedback, self.linkage_fb_cb
        )        

    def linkage_fb_cb(self, msg):

        time_stamp = rospy.get_time() - self.time_last
        print('{:.2f}, {}, {}, {}, {}\n'.format(time_stamp, msg.motor_current, msg.measured_velocity, msg.supply_voltage, msg.supply_current))
        f = open(feedback_file, 'a')
        f.write('{:.2f}, {}, {}, {}, {}\n'.format(time_stamp, msg.motor_current, msg.measured_velocity, msg.supply_voltage, msg.supply_current))
        f.close()


    def run(self):
        rate = rospy.Rate(100)
        self.time_last = rospy.get_time()
        print (self.time_last)


        while not rospy.is_shutdown():
            rate.sleep()

if __name__ == '__main__':
    Listener().run()
