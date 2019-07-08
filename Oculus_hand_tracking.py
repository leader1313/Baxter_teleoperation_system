#!/usr/bin/env python
# -*- coding: utf-8 -*-

# 2019.06.21 Oculus with Baxter
# Oh hanbit
#
# baxter_simulator/ik_pick_and_place_demo.py を改変（元ソースはggれば出る）
# Robotiq_Gripperの使用には別途準備が必要（readme.txtを参照）
#
# Baxter正面から見て横方向軸：y軸
# Baxter正面から見て奥行き方向軸：x軸　と読み替えているので注意（元々はxyが逆）

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
# import baxter_tools

class HandTracking(object):
    def __init__(self, limb,verbose=True):
        self._limb_name = limb  # string
        self._verbose = verbose  # bool
        self._limb = baxter_interface.Limb(limb)
        # self._tuck = baxter_tools.Tuck(tuck_cmd)
        # self._movetuck = _tuck(self.current_joy.buttons[5])
        self._limb.set_joint_position_speed(1.0)
        self._jn_list = self._limb.joint_names()
        self._w2_joint_name = limb + '_w2'
        self._gripper = baxter_interface.Gripper(limb)  # baxter default gripper
        self.robotiq_gripper_pub = rospy.Publisher(
            '/robotiq/gripper_' + limb + '/command/position', Int32, queue_size=10) 
        self.robotiq_gripper_force_pub = rospy.Publisher(
            '/robotiq/gripper_' + limb + '/command/force', Int32, queue_size=10)
        self.robotiq_gripper_speed_pub = rospy.Publisher(
            '/robotiq/gripper_' + limb + '/command/speed', Int32, queue_size=10)
        ns = 'ExternalTools/' + limb + '/PositionKinematicsNode/IKService'
        self._iksvc = rospy.ServiceProxy(ns, SolvePositionIK)
        rospy.wait_for_service(ns, 5.0)
        # verify robot is enabled
        print('Getting robot state... ')
        self._rs = baxter_interface.RobotEnable(baxter_interface.CHECK_VERSION)
        self._init_state = self._rs.state().enabled
        print('Enabling robot... ')
        self._rs.enable()

        self.Oculus_hand_sub = rospy.Subscriber(
            '/Oculus_Hand_Pose_'+ limb, PoseStamped, self._callback_signal)
        self.Oculus_joy_sub = rospy.Subscriber(
            '/Oculus_Joy', Joy, self._callback_joy)
        self.current_pose = self._limb.endpoint_pose()
        self.current_joy = Joy()
       
        self.default_orientation = Quaternion(
            x=-0.0249590815779,
            y=0.999649402929,
            z=0.00737916180073,
            w=0.00486450832011
        )
        self.starting_position = [
            0.68520,
            0.27624,
            0.2
        ]
        self.finishing_position = [
            0.68263,
            0.62523,
            0.25
        ]
        if limb == 'right':     # 横方向座標を逆転
            self.starting_position[1]  *= -1.0
            self.finishing_position[1] *= -1.0


 # Oculusから信号が来たらself.current_poseに格納する関数
    def _callback_signal(self, data):
        self.current_pose = data.pose
        # print(data.pose)

 # Oculusから信号が来たらself.current_poseに格納する関数
    def _callback_joy(self, data):
        self.current_joy = data
        # print(data)


    def move_to_start_pos(self):
        print('Moving the {0} arm to start pose...'.format(self._limb_name))
        sp = self.starting_position
        do = self.default_orientation
        self.changePosePosition(sp[0], sp[1], sp[2], do)
        
        self.gripperCalibrate()
        self.gripperClose()
        # self.rotateWristAngleDeg(0.0)
        print('\033[32m' + 'Enjoy candy picking with Baxter! Ctrl-c to quit' + '\033[0m')
        

 # 逆運動学(ik)を解く関数（何やってるかあんまりわからん）
    def ik_request(self, pose):
        hdr = Header(stamp=rospy.Time.now(), frame_id='base')
        ikreq = SolvePositionIKRequest()
        ikreq.pose_stamp.append(PoseStamped(header=hdr, pose=pose))
        try:
            resp = self._iksvc(ikreq)
        except (rospy.ServiceException, rospy.ROSException), e:
            rospy.logerr('Service call failed: %s' % (e,))
            return False
        # Check if result valid, and type of seed ultimately used to get solution
        # convert rospy's string representation of uint8[]'s to int's
        resp_seeds = struct.unpack('<%dB' % len(
            resp.result_type), resp.result_type)
        limb_joints = {}
        if (resp_seeds[0] != resp.RESULT_INVALID):
            seed_str = {
                ikreq.SEED_USER: 'User Provided Seed',
                ikreq.SEED_CURRENT: 'Current Joint Angles',
                ikreq.SEED_NS_MAP: 'Nullspace Setpoints',
            }.get(resp_seeds[0], 'None')
            # Format solution into Limb API-compatible dictionary
            limb_joints = dict(
                zip(resp.joints[0].name, resp.joints[0].position))
        else:
            rospy.logerr('INVALID POSE - No Valid Joint Solution Found.')
            return False
        return limb_joints

    # エラー処理付きのmove_to_joint_positions関数
    def _guarded_move_to_joint_position(self, joint_angles):
        if joint_angles:
            self._limb.move_to_joint_positions(joint_angles)
        else:
            rospy.logerr(
                'No Joint Angles provided for move_to_joint_positions. Staying put.')

    def gripperCalibrate(self):
        print 'gripper calibrate'
        if self._limb_name=='right':
            self._gripper.calibrate()  
        rospy.sleep(2.0)
        
    def gripperOpen(self):
        print 'gripper open'
        if self._limb_name=='right':
            self._gripper.open()   # inverted
        else:
           self.robotiq_gripper_pub.publish(0)
        rospy.sleep(2.0)

    def gripperClose(self):
        # self._gripper.close()
        print 'gripper close'
        if self._limb_name=='right':
            self._gripper.close()
        else:
           self.robotiq_gripper_force_pub.publish(1)
           self.robotiq_gripper_speed_pub.publish(255)
        #    self.robotiq_gripper_pub.publish(220)   # nip
           self.robotiq_gripper_pub.publish(220)   # scoop
        rospy.sleep(2.0)

    def rotateWristAngleDeg(self, angle_deg):
        angle_rad = angle_deg * 3.14159 / 180.0
        self._limb.set_joint_positions({self._w2_joint_name: angle_rad})
        # print('change wrist angle to ' + str(angle_deg) +
        #       '[deg] = ' + str(angle_rad) + '[rad]')

    def tuck_arm(self):
        subprocess.call(["rosrun","baxter_tools","tuck_arms.py","-u"])


    def decideWristAngle(self):
        add_angle_deg = 10.0
        max_angle_deg = 60.0
        min_angle_deg = -60.0
        limb_name = self._limb_name

        while not rospy.is_shutdown():
            current_angle_rad = self._limb.joint_angle(self._w2_joint_name)
            current_angle_deg = current_angle_rad * 180.0 / 3.14159
            next_angle_deg = current_angle_deg
            cj = self.current_joy.buttons

            if limb_name == 'right' :
                if self.current_joy.buttons[5] == 1:
                    break
                if self.current_joy.axes[0]>0.5 :
                    self.gripperClose()
                elif self.current_joy.axes[1]>0.5 :
                    self.gripperOpen()
                if cj[0] == 1:
                    next_angle_deg = current_angle_deg - add_angle_deg
                elif cj[1] == 1:
                    next_angle_deg = current_angle_deg + add_angle_deg
            else:
                if self.current_joy.buttons[4] == 1:
                    break
                if self.current_joy.axes[2]>0.5 :
                    self.gripperClose()
                elif self.current_joy.axes[3]>0.5 :
                    self.gripperOpen()
                if cj[2] == 1:
                    next_angle_deg = current_angle_deg - add_angle_deg
                elif cj[3] == 1:
                    next_angle_deg = current_angle_deg + add_angle_deg
            if self.current_joy.buttons[6] == 1:
                self.tuck_arm()
            self.rotateWristAngleDeg(next_angle_deg)
            
            rospy.sleep(1.0)

    def changePosePosition(self, px, py, pz, orient):
        new_pose = Pose()

        new_pose.position.x = px
        new_pose.position.y = py
        new_pose.position.z = pz

        new_pose.orientation = orient
        
         #7 DOF(degree of freedom) Angle
        joint_angles = self.ik_request(new_pose)
        self._guarded_move_to_joint_position(joint_angles)
        rospy.sleep(0.1)


	def pickCandys(self, z_hover, z_hover_2, z_pick):
		# 手首角度選択へ
		current_x = current_pose['position'].x
		current_y = current_pose['position'].y

		do = self.default_orientation
		self.changePosePosition(
		    current_x,
		    current_y,
		    z_hover_2,
		    do #default_orientation
		)

		
	#姿勢Quaternion(四元数)、roll yaw pitch　変換できる
		# Candyを掴む
		self.changePosePosition(
		    current_x,
		    current_y,
		    z_pick,
		    self._limb.endpoint_pose()['orientation']
		)
		self.gripperClose()

	#        # 真上に戻る
	#        self.changePosePosition(
	#            current_x,
	#            current_y,
	#            z_hover,
	#            do
	#        )

		# カゴへ移動
		fp = self.finishing_position
		self.changePosePosition(fp[0], fp[1], fp[2], do)
		self.gripperOpen()

		# 初期位置へ移動
		self.move_to_start_pos()
    
    def movingHand(self):

        # maxMin_dict = {
        #     'x_max_left':   0.50,
        #     'x_min_left':   0.35,
        #     'x_max_right': -0.35,
        #     'x_min_right': -0.50,
        # }
				
        limb_name = self._limb_name
        # x_max = maxMin_dict['x_max_' + limb_name]
        # x_min = maxMin_dict['x_min_' + limb_name]
        # y_max = 0.7
        # y_min = 0.5
        # z_hover = 0.20
        # z_hover_2 = 0.10
        # # z_pick = 0.055
        # z_pick = -0.025

        while 1:
            
            self.decideWristAngle()
            self.changePosePosition(
                    self.current_pose.position.x ,
                    self.current_pose.position.y-0.2 ,
                    self.current_pose.position.z+0.15 ,
                    self.default_orientation
                    # self.current_pose.orientation
                )

            rospy.sleep(0.1)
            

def load_gazebo_models(table_pose=Pose(position=Point(x=1.0, y=0.0, z=0.0)),
                       table_reference_frame="world",
                       block_pose=Pose(position=Point(x=0.6725, y=0.1265, z=0.7825)),
                       block_reference_frame="world"):
    # Get Models' Path
    model_path = rospkg.RosPack().get_path('baxter_sim_examples')+"/models/"
    # Load Table SDF
    table_xml = ''
    with open (model_path + "cafe_table/model.sdf", "r") as table_file:
        table_xml=table_file.read().replace('\n', '')
    # Load Block URDF
    block_xml = ''
    with open (model_path + "block/model.urdf", "r") as block_file:
        block_xml=block_file.read().replace('\n', '')
    # Spawn Table SDF
    rospy.wait_for_service('/gazebo/spawn_sdf_model')
    try:
        spawn_sdf = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)
        resp_sdf = spawn_sdf("cafe_table", table_xml, "/",
                             table_pose, table_reference_frame)
    except rospy.ServiceException, e:
        rospy.logerr("Spawn SDF service call failed: {0}".format(e))
    # Spawn Block URDF
    rospy.wait_for_service('/gazebo/spawn_urdf_model')
    try:
        spawn_urdf = rospy.ServiceProxy('/gazebo/spawn_urdf_model', SpawnModel)
        resp_urdf = spawn_urdf("block", block_xml, "/",
                               block_pose, block_reference_frame)
    except rospy.ServiceException, e:
        rospy.logerr("Spawn URDF service call failed: {0}".format(e))

def delete_gazebo_models():
    # This will be called on ROS Exit, deleting Gazebo models
    # Do not wait for the Gazebo Delete Model service, since
    # Gazebo should already be running. If the service is not
    # available since Gazebo has been killed, it is fine to error out
    try:
        delete_model = rospy.ServiceProxy('/gazebo/delete_model', DeleteModel)
        resp_delete = delete_model("cafe_table")
        resp_delete = delete_model("block")
    except rospy.ServiceException, e:
        rospy.loginfo("Delete Model service call failed: {0}".format(e))


def main():
    # print 'Waiting for the All Clear from emulator startup... '
    # rospy.wait_for_message('/robot/sim/started', Empty)
    #load_gazebo_models()
    # Remove models from the scene on shutdown
    #rospy.on_shutdown(delete_gazebo_models)
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
    rospy.init_node('ik_hand_tracking_' + limb)
    HT = HandTracking(limb)
    
    print 'Move to starting angles...'
    HT.move_to_start_pos()
    while not rospy.is_shutdown():
        HT.movingHand()
        rospy.sleep(0.1)
    return 0
        
    

if __name__ == '__main__':
    sys.exit(main())
