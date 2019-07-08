#!/usr/bin/env python

# Software License Agreement (BSD License)
#
# Copyright (c) 2013, SRI International
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of SRI International nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# Author: Acorn Pooley

# BEGIN_SUB_TUTORIAL imports
#
# To use the python interface to move_group, import the moveit_commander
# module.  We also import rospy and some messages that we will use.
import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
# END_SUB_TUTORIAL

from std_msgs.msg import String


def move_group_python_interface_baxter():
    # BEGIN_TUTORIAL
    #
    # Setup
    # ^^^^^
    # CALL_SUB_TUTORIAL imports
    #
    # First initialize moveit_commander and rospy.
    print "============ Starting tutorial setup"
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('move_group_python_interface_baxter',
                    anonymous=True)

    robot = moveit_commander.RobotCommander()    # not working
    scene = moveit_commander.PlanningSceneInterface()
    lArm = moveit_commander.MoveGroupCommander("left_arm")

    print "============ Waiting for RVIZ..."
    rospy.sleep(2)
    print "============ Starting tutorial "

    print "============ Reference frame: %s" % lArm.get_planning_frame()
    print "============ End-effector link: %s" % lArm.get_end_effector_link()
    print "============ Robot Groups:"
    print robot.get_group_names()
    print "============ Printing robot state"
    print robot.get_current_state()
    print "============"

    # Planning to a Pose goal
    # ^^^^^^^^^^^^^^^^^^^^^^^
    # We can plan a motion for this group to a desired pose for the
    # end-effector
    print "============ Pose setting..."
    pose_target = lArm.get_current_pose().pose
    # joint = lArm.get_current_joint_values()

    print "before"
    print "pose= " + str(pose_target)
    # print "joint= " + str(joint)

    pose_target.position.x += 1.0
    pose_target.position.y += 1.0
    pose_target.position.z += 1.0

    print "after"
    print "pose= " + str(pose_target)
    # print "joint= " + str(joint)

    lArm.set_pose_target(pose_target)

    print "============ Planning and executing..."
    plan1 = lArm.plan()
    lArm.execute(plan1)

    rospy.sleep(1)
    lArm.stop()
    lArm.clear_pose_targets()

    print "============ shutdown"
    # When finished shut down moveit_commander.
    moveit_commander.roscpp_shutdown()

    # END_TUTORIAL
    print "============ STOPPING"


if __name__ == '__main__':
    try:
        move_group_python_interface_baxter()
    except rospy.ROSInterruptException:
        pass
