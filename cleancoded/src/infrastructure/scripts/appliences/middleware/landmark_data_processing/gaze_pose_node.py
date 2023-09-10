# This node is responsible for calling the gaze_pose service and sending the audio data to it.
#!/usr/bin/env python

import rospy
from infrastructure.msg import List
from infrastructure.srv import Gaze
from std_msgs.msg import String



rospy.init_node('gaze_pose_node',anonymous=False)
pub = rospy.Publisher('gaze_position/gaze_dir', String, queue_size=10)


def landmark_handler(data_list):
    global landmarks
    landmarks = data_list
    rospy.Subscriber('/image_cv2',List, callservice)


def callservice(frame):
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
    rospy.Subscriber("/image_cv2/landmarked", List, landmark_handler)
    rospy.spin()