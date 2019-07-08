#!/usr/bin/python
# -*- coding: utf-8 -*-

import numpy as np
import sys
from time import sleep

import rospy
from std_msgs.msg import Int32
from std_msgs.msg import Int32MultiArray
from std_msgs.msg import Float32
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import Bool


def start_sim(pub):
    print 'start simulation'
    init_rate = rospy.Rate(10)
    frag = Bool()
    frag.data = True
    for _ in range(10):
        init_rate.sleep()
        pub.publish(frag)


def stop_sim(pub):
    print 'stop simulation'
    frag = Bool()
    frag.data = True
    pub.publish(frag)


# Subscribeする対象のトピックが更新されたら呼び出されるコールバック関数
# 引数にはトピックにPublishされるメッセージの型と同じ型を定義する
def callback_cyl(data, sub):
    # 受けとったmessageの中身を出力
    print "x:", data.data[0]
    print "y:", data.data[1]
    print "z:", data.data[2]

    return data.data


def func_ik(ik_l, pub):
    r = rospy.Rate(10)

    wait = 20
    for _ in range(wait):
        # ik.data = [i, 0.0, 0.0]
        # publishする関数
        pub.publish(ik_l)

        # print "ik data ", ik_l.data
        r.sleep()


def func_gripVel(grip_l, pub):
    r = rospy.Rate(10)

    wait = 20
    for _ in range(wait):
        # ik.data = [i, 0.0, 0.0]
        # publishする関数
        pub.publish(grip_l)

        # print "grip data", grip_l.data
        r.sleep()


def grip():
    rospy.init_node('baxter_grip', disable_signals=True)

    start_sim_pub = rospy.Publisher('startSimulation', Bool, queue_size=10)
    stop_sim_pub = rospy.Publisher('stopSimulation', Bool, queue_size=10)

    start_sim(start_sim_pub)

    ik_target_pub = rospy.Publisher(
        'IKTarget', Float32MultiArray, queue_size=10)
    grip_vel_pub = rospy.Publisher('GripVel', Float32, queue_size=10)
    # r = rospy.Rate(10)

    cyl_state_sub = None
    cyl_state_sub = rospy.Subscriber(
        'cyl_state', Float32MultiArray, callback_cyl, cyl_state_sub)

    # Cylinder Location :
    # x : 0.05
    # y : 0.35
    # z : 0.82362

    # Float32MultiArray()型のikインスタンス
    ik = Float32MultiArray()

    grip = Float32()

    z = [1.2, 0.95, 0.9]
    # i=0.0

    #
    cylLocation = [-0.4, 0.375, 0.82362]
    holeLocation = [-0.2, 0.375, 0.82362]
    gripVel = 0.01

    ik_target_pub.publish(ik)
    print "Location was published"

    # ターゲットの真上へ移動
    ik.data = [cylLocation[0], cylLocation[1], 1.2]
    func_ik(ik, ik_target_pub)

    # 降下
    ik.data = [cylLocation[0], cylLocation[1], 0.94]
    func_ik(ik, ik_target_pub)

    ik.data = [cylLocation[0], cylLocation[1], 0.87]
    func_ik(ik, ik_target_pub)

    # グリッパで掴む
    grip.data = gripVel
    func_gripVel(grip, grip_vel_pub)

    # 穴の真上へ移動
    ik.data = [holeLocation[0], holeLocation[1], 1.2]
    func_ik(ik, ik_target_pub)

    # 降下
    ik.data = [holeLocation[0], holeLocation[1], 1.0]
    func_ik(ik, ik_target_pub)

    ik.data = [holeLocation[0], holeLocation[1], 0.93]
    func_ik(ik, ik_target_pub)

    # グリッパで離す
    grip.data = -gripVel
    func_gripVel(grip, grip_vel_pub)

    # 真上へ移動
    ik.data = [holeLocation[0], holeLocation[1], 1.2]
    func_ik(ik, ik_target_pub)

    stop_sim(stop_sim_pub)


if __name__ == '__main__':
    try:
        grip()
        # sleep(5)
    except rospy.ROSInterruptException:
        print "Exception"
        pass
