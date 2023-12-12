#!/usr/bin/env python3
import rospy
from infrastructure.msg import FaceEmotions, Exp

class ExpUpdater:
    def __init__(self):
        self.pub = rospy.Publisher('py_exp_publisher', Exp, queue_size=10)
        rospy.init_node('exp_updater', anonymous=True)
        self.rate = rospy.Rate(10)
        rospy.loginfo("Successfully subscribed to /face_emotions. Sending dominant emotion to /py_exp_publisher")
        rospy.Subscriber("/face_emotions", FaceEmotions, self.callback)

    def callback(self, data):
        data_list = data.data
        # get the emotion with the highest probability
        highest_prob = 0
        for i in range(len(data_list)):
            if data_list[i].probability > highest_prob:
                highest_prob = data_list[i].probability
                feeling = data_list[i].emotion
        rospy.log_info("feeling is ", feeling)

        # publish the emotion
        exp = Exp()
        exp.emotion = feeling
        exp.auto_imit = True
        exp.time = rospy.get_time()
        self.pub.publish(exp)