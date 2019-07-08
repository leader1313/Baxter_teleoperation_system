#!/usr/bin/python2

import os
import sys
import argparse

import rospy

import cv2
import cv_bridge

from sensor_msgs.msg import (
    Image,
)


def send_image(path_list):
    """
    Send the image located at the specified path to the head
    display on Baxter.

    @param path: path to the image file to load and send
    """
    pub = rospy.Publisher('/robot/xdisplay', Image, queue_size=10)

    msg_list = []

    scale = 1024.0/720.0

    for path in path_list:
        print(path)
        img = cv2.imread(path)
        reImg = cv2.resize(img, None, fx=scale, fy=scale)
        msg_list.append(cv_bridge.CvBridge().cv2_to_imgmsg(reImg, encoding="bgr8"))

    try:
        for msg in msg_list:
            pub.publish(msg)
            rospy.sleep(1.0)
    except KeyboardInterrupt:
        print('Interrupted')
        sys.exit()


def main():
    rospy.init_node('rsdk_xdisplay_image', anonymous=True)

    file_dir = './face_images/'
    file_list = []
    for i in os.listdir(file_dir):
        if not os.path.isdir(file_dir + i):
            file_list.append(i)

    file_name_head = file_list[0].split('_')[0]
    file_ext = '.' + file_list[0].split('.')[1]

    loop = len(file_list) - 1

    file_path_list = []

    for j in range(loop, -1, -1):
        file_name = file_name_head + '_' + str(j).zfill(2)
        file_path_list.append(file_dir + file_name + file_ext)

    send_image(file_path_list)

    return 0


if __name__ == '__main__':
    sys.exit(main())
    # main()
