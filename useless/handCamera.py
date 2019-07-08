#!/usr/bin/python2

import os
import sys
import argparse

import rospy
import cv2
from cv_bridge import CvBridge, CvBridgeError

from sensor_msgs.msg import (
    Image,
)
from geometry_msgs.msg import (
    Pose,
    Point,
)
from baxter_core_msgs.msg import (
    EndpointState,
)

TAN_ALPHA = 0.743
D0 = 0.035
OFFSET = 0.15

class handCamera():
    def __init__(self, limb):
        self.limb = limb
        self.bridge = CvBridge()
        self.sub_img = rospy.Subscriber('/cameras/' + limb + '_hand_camera/image', Image, self.imageCallback)
        self.sub_h0 = rospy.Subscriber('/robot/limb/' + limb + '/endpoint_state', EndpointState, self.heightCallback)
        self.h0 = 0.20
        self.h  = self.h0

    def imageCallback(self, data):
        # print(self.limb + ': ' + str(self.h))

        try:
            cv_image_raw = self.bridge.imgmsg_to_cv2(data, "bgr8")

        except CvBridgeError as e:
            print('Cv_Bridge_Error: ' + e)

        if len(cv_image_raw.shape) == 3:
            imgHeight, imgWidth, channels = cv_image_raw.shape[:3]
        else:
            imgHeight, imgWidth = cv_image_raw.shape[:2]
            channels = 1

        rotationMat = cv2.getRotationMatrix2D((imgWidth/2, imgHeight/2), 180, 1)
        cv_image = cv2.warpAffine(cv_image_raw, rotationMat, (imgWidth, imgHeight))

        offset = {
            'left' : [ 30, 100],
            'right': [-30, 100],
        }

        redPointX = imgWidth/2# + offset[self.limb][0]
        redPointY = imgHeight/2# + offset[self.limb][1]
        
        # redPointY = int( (imgWidth*D0) / (2*TAN_ALPHA*(self.h+OFFSET)) )

        # if self.limb=='left':
            # print(redPointX, redPointY)

        # img, center, radius, color, thickness=1, lineType=cv2.LINE_AA, shift=0    
        cv2.circle(cv_image, (redPointX, redPointY), 8, (0, 0, 255), thickness=-1)#, lineType=cv2.CV_AA)
        
        cv2.imshow("Origin Image " + self.limb, cv_image)
        cv2.waitKey(1)
        # rospy.sleep(1)

    def heightCallback(self, data):
        self.h = data.pose.position.z

def main(args):
    rospy.init_node('handCameraViewNode')

    hC_l = handCamera('left')
    hC_r = handCamera('right')

    try:
        rospy.spin()

    except KeyboardInterrupt:
        print("Shutting down")
        cv2.destroyAllWindows()


if __name__ == '__main__':
    main(sys.argv)
