#!/usr/bin/python
# -*- coding: utf-8 -*-

import rospy
from std_msgs.msg import Int32
import baxter_external_devices


signal = -1


def _callback_signal(data):
    global signal
    print(data.data)
    signal = data.data


if __name__ == '__main__':
    limb_dict = {
        'l': 'left',
        'r': 'right'
    }

    armArg = 'a'

    while not armArg in ['l', 'r']:
        print 'Which arm? (press l or r)'
        armArg = raw_input()

    limb = limb_dict[armArg]

    print limb

    rospy.init_node('baxter_kinect_listener_'+limb, anonymous=True)
    arms_signals_sub = rospy.Subscriber('arms_signals_' + limb, Int32, _callback_signal)
    r = rospy.Rate(100)

    print "-- Waiting the subscribed command --"

    cmd_list = ["0", "1", "2", "3", "4", "5"]

    global signal

    while not rospy.is_shutdown():
        if str(signal) in cmd_list:
            print "\n**********************"
            print limb, str(signal), "is subscribed"
            print "**********************\n"
        else:
            pass
        # if str(signal) and not str(signal) in cmd_list:
        #     print "\n**********************"
        #     print "no command"
        #     print "**********************\n"
        r.sleep()
