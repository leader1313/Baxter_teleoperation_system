#!/usr/bin/python
# -*- coding: utf-8 -*-

import rospy
from std_msgs.msg import Int32
# from std_msgs.msg import Int32MultiArray
# from std_msgs.msg import Float32
# from std_msgs.msg import Float32MultiArray
# from std_msgs.msg import Bool
import baxter_external_devices

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

    rospy.init_node('baxter_kinect_test_'+limb, anonymous=True)
    arms_signals_pub = rospy.Publisher('arms_signals_'+limb, Int32, queue_size=10)
    r = rospy.Rate(100)

    print "-- Enter the publishing command below --"

    cmd_list = ["0", "1", "2", "3", "4", "5"]

    signal = 0

    while not rospy.is_shutdown():
        c = baxter_external_devices.getch()
        if c in cmd_list:
            print "\n**********************"
            signal = int(c)
            arms_signals_pub.publish(signal)
            print limb, str(signal), "is published"
            print "**********************\n"
        elif c and not c in cmd_list:
            print "\n**********************"
            print "no command"
            print "**********************\n"
        r.sleep()
