#!/usr/bin/env python3
import rospy
from infrastructure.msg import FaceEmotions, Exp

class ExpUpdater:
    def __init__(self):
        self.count = 0
        self.pub = rospy.Publisher('py_exp_publisher', Exp, queue_size=10)
        rospy.init_node('exp_updater', anonymous=True)
        self.rate = rospy.Rate(10)
        rospy.loginfo("Successfully subscribed to /face_emotions. Sending dominant emotion to /py_exp_publisher. Check the web interface for the robot's reaction.")
        rospy.Subscriber("/face_emotions", FaceEmotions, self.callback)

    def callback(self, data):
        self.count+=1
        if self.count%10 == 0: # slow down the rate of publishing to let the robot react
            rospy.loginfo("Received face emotion data")
            self.sendexp(data)

    def sendexp(self, data):
        data_list = data.data
        # get the emotion with the highest probability
        highest_prob = 0
        for i in range(len(data_list)):
            if data_list[i].probability > highest_prob:
                highest_prob = data_list[i].probability
                feeling = data_list[i].emotion
        rospy.loginfo("dominant emotion is "+ feeling)

        # publish the emotion
        exp = Exp()
        exp.emotion = feeling
        exp.auto_imit = True
        exp.time = rospy.get_time()
        self.pub.publish(exp)


if __name__ == '__main__':
    try:
        ExpUpdater()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass