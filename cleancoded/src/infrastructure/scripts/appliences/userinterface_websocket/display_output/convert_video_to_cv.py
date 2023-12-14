#!/usr/bin/env python3

# this node subscribes to /py_exp_publisher, based on the exp value, opens the video file [exp value].mp4, converts it to cv2 format and publishes it to /cv2_exp_publisher

import rospy
import cv2
from std_msgs.msg import Int32
from sensor_msgs.msg import Image
from infrastructure.msg import Exp
import numpy as np
import os

class ConvertVideoToCv:
    height = 640
    width = 480	
    def __init__(self):
        rospy.init_node('convert_video_to_cv', anonymous=True)
        self.pub = rospy.Publisher('/cv2_exp_publisher', Image, queue_size=10)
        self.sub = rospy.Subscriber('/py_exp_publisher', Exp, self.callback)
        self.rate = rospy.Rate(10)
        self.exp = 0
        self.path = os.path.dirname(os.path.realpath(__file__)) + "/../videos/"

    def callback(self, data):
        self.exp = data.data

    def convert_back(self,cv2_img):
        ros_image = Image()
        ros_image.header.stamp = rospy.Time.now()
        ros_image.height = cv2_img.shape[0]
        ros_image.width = cv2_img.shape[1]
        ros_image.encoding = "bgr8"
        ros_image.step = cv2_img.shape[1] * 3
        ros_image.data = np.array(cv2_img).tobytes()		
        return ros_image


    def run(self):
        while not rospy.is_shutdown():
            try:
                img = cv2.imread(self.path + str(self.exp) + ".mp4")
                img = cv2.resize(img, (self.height, self.width))
                img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
                img = self.convert_back(img)
                self.pub.publish(img)
            except BaseException as e:
                print(e)
            self.rate.sleep()


if __name__ == '__main__':
    try:
        cv = ConvertVideoToCv()
        cv.run()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
