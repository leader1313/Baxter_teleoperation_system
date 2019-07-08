#!/usr/bin/python
# -*- coding: utf-8 -*-

import rospy
from sensor_msgs.msg import Joy
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
    error_pub = rospy.Publisher(
            '/error_' + limb + '_hand', Joy, queue_size=10) 
    r = rospy.Rate(100)

    error = Joy()
    error.header.frame_id = 'Unity'
    error.buttons = [1,1]
    print "-- Enter the publishing command below --"

    # cmd_list = ["0", "1", "2", "3", "4", "5"]

    # signal = 0

    while not rospy.is_shutdown():
        error_pub.publish(error)
        print error.buttons[0]
        

        r.sleep()
