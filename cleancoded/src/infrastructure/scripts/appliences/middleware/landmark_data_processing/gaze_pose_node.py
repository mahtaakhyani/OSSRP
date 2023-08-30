# This node is responsible for calling the gaze_pose service and sending the audio data to it.
#!/usr/bin/env python

import rospy
from std_msgs.msg import List
from infrastructure.msg import Gaze



rospy.init_node('gaze_pose_node',anonymous=False)
pub = rospy.Publisher('gaze_position/gaze_dir', Gaze, queue_size=10)

def callservice(data):
    rospy.wait_for_service('gaze_pose')
    try:
        gazsrv = rospy.ServiceProxy('gaze_pose', Gaze)
        transcript = gazsrv(data.gazedirection)
        rospy.loginfo("gaze_pose server successfully responded with %s",transcript)
        rospy.loginfo("Publishing gaze position to /gaze_position/gaze_dir topic")
        pub.publish(transcript)
        
    except rospy.ServiceException as e:
        rospy.logerr("Service call failed: %s"%e)


if __name__ == '__main__':
    rospy.Subscriber('/image_cv2',List, callservice)
    rospy.spin()