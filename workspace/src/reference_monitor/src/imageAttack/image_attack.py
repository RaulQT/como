#!/usr/bin/env python

import numpy as np
import cv2
import rospy
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image



class ImagePublisher:
    def __init__(self):
        self.bridge = CvBridge()
        self.pub_recon = rospy.Publisher("/cam/raw", Image, queue_size=1)

    def pub_imgs(self, img_edit):
        try:
           # self.pub_recon.publish(cv2.imread("/home/odroid/.ros/camera_image.jpeg",0))
           self.pub_recon.publish(self.bridge.cv2_to_imgmsg(img_edit, "mono8"))
        except CvBridgeError as e:
            print(e)

def image_callback(msg):
    bridge = CvBridge()
    try:
        img = bridge.imgmsg_to_cv2(msg, "mono8")
    except CvBridgeError, e:
        print(e)
    img2 = cv2.imread("/home/odroid/.ros/camera_image.jpeg",0)
    ret, img_edit2 = cv2.threshold(img2, 255, 255, cv2.THRESH_BINARY)

    ip = ImagePublisher()
    ip.pub_imgs(img_edit2)

def main():

    rospy.init_node('image_attack')

    rate = rospy.Rate(10)

    image_topic = "/cam/raw"
    rospy.Subscriber(image_topic, Image, image_callback)
    
    rospy.spin()
    rate.sleep()

if __name__ == '__main__':
    main()
