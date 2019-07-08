#!/usr/bin/python
# -*- coding: utf-8 -*-
import argparse
import struct
import sys
import copy
import numpy as np

import subprocess
import actionlib
import rospy
import rospkg
from std_msgs.msg import Int32

from control_msgs.msg import (
    SingleJointPositionAction,
    SingleJointPositionGoal,
)
from gazebo_msgs.srv import (
    SpawnModel,
    DeleteModel,
)
from geometry_msgs.msg import (
    PoseStamped,
    Pose,
    Point,
    Quaternion,
)
from std_msgs.msg import (
    Header,
    Empty,
)
from sensor_msgs.msg import Joy

from baxter_core_msgs.srv import (
    SolvePositionIK,
    SolvePositionIKRequest,
)

import baxter_interface


class posestamped_publisher(object):
    def __init__(self, limb):
        self._limb_name = limb  # string     
        self._limb = baxter_interface.Limb(limb)
        self._PoseStamped=PoseStamped()
        self.pose_pub = rospy.Publisher(
            '/Baxter/' + limb + '_hand', PoseStamped, queue_size=10) 
    
    def Publisher(self):
        # self._PoseStamped.header.frame_id='Baxter'
        self._PoseStamped.pose.position.x=self._limb.endpoint_pose()['position'].x
        self._PoseStamped.pose.position.y=self._limb.endpoint_pose()['position'].y
        self._PoseStamped.pose.position.z=self._limb.endpoint_pose()['position'].z
        self._PoseStamped.pose.orientation.x=self._limb.endpoint_pose()['orientation'].x
        self._PoseStamped.pose.orientation.y=self._limb.endpoint_pose()['orientation'].y
        self._PoseStamped.pose.orientation.z=self._limb.endpoint_pose()['orientation'].z
        self._PoseStamped.pose.orientation.w=self._limb.endpoint_pose()['orientation'].w
        self.pose_pub.publish(self._PoseStamped)

def main():
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
    print 'Initializing node...'
    rospy.init_node('baxter_' + limb, anonymous=True)
    HT = posestamped_publisher(limb)
    
    print 'Move to starting angles...'
    
    while not rospy.is_shutdown():
        HT.Publisher()
        print(HT._PoseStamped)
        rospy.sleep(0.1)
    return 0
if __name__ == '__main__':
     sys.exit(main())