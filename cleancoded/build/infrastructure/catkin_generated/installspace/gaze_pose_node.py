#!/usr/bin/env python2
# This node is responsible for calling the gaze_pose service and sending the audio data to it.

import rospy
from infrastructure.msg import List, Landmarks
from infrastructure.srv import Gaze
from std_msgs.msg import String
from sensor_msgs.msg import Image



rospy.init_node('gaze_pose_node',anonymous=False)
pub = rospy.Publisher('gaze_position/gaze_dir', String, queue_size=10)


def landmark_handler(data_list):
    global landmarks
    landmarks = data_list
    rospy.loginfo('landmarks recieved')
    # rospy.Subscriber('/image_cv2',List, callservice)
    rospy.Subscriber("/image_raw", Image, callservice, callback_args=False, queue_size=1, buff_size=2**19) 



def callservice(frame,_):
    rospy.loginfo("waiting for gaze pose service to respond...")
    rospy.wait_for_service('gaze_pose')
    try:
        gazsrv = rospy.ServiceProxy('gaze_pose', Gaze)
        rospy.loginfo("gaze pose service responded.")
        transcript = gazsrv(frame, landmarks)
        rospy.loginfo("gaze_pose server successfully responded with %s",transcript.gazedirection)
        rospy.loginfo("Publishing gaze position to /gaze_position/gaze_dir topic")
        pub.publish(transcript.gazedirection)
        
    except rospy.ServiceException as e:
        rospy.logerr("Service call failed: %s"%e)



if __name__ == '__main__':
    rospy.Subscriber("/landmarks", Landmarks, landmark_handler)
    rospy.spin()