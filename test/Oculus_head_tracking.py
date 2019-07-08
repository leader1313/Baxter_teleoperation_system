import argparse
import struct
import sys
import copy
import tf
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
    Vector3,
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


class Oculus_head_tracking(object):
    def __init__(self):
        self._head = baxter_interface.Head()
        self._PoseStamped=PoseStamped()
        self.head_pose = Quaternion()
        self.Oculus_head_sub = rospy.Subscriber(
            '/Oculus_head_move', PoseStamped, self.callback_head_pose)
    
    def callback_head_pose(self,data):
        self.head_pose = data.pose.orientation


def quaternion_to_euler(quaternion):
    """Convert Quaternion to Euler Angles

    quarternion: geometry_msgs/Quaternion
    euler: geometry_msgs/Vector3
    """
    e = tf.transformations.euler_from_quaternion((quaternion.x, quaternion.y, quaternion.z, quaternion.w))
    return Vector3(x=e[0], y=e[1], z=e[2])



def main():

    print 'Initializing node...'
    rospy.init_node('Oculus_head', anonymous=True)
    HT = Oculus_head_tracking()
    
    print 'Move to starting angles...'
    
    while not rospy.is_shutdown():
        E = quaternion_to_euler(HT.head_pose)
        HT._head.set_pan(E.z)
        print(HT.head_pose)
        print(E.x)
        rospy.sleep(0.1)
    return 0
if __name__ == '__main__':
     sys.exit(main())