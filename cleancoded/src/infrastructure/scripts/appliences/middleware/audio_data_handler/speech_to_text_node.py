# This node is responsible for calling the speech_to_text service and sending the audio data to it.
# the pioneer node that subscribes to the audio data topic and send the audio data to speech to text server and receives the transcript as a String.
# Then it initiates a topic to publish the received transcript to the topic /transcript as a String.
#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from infrastructure.msg import Stt
from audio_common_msgs.msg import AudioDataStamped as AudioData



rospy.init_node('speech_to_text_node',anonymous=False)
pub = rospy.Publisher('transcript', String, queue_size=10)

def callservice(data):
    rospy.wait_for_service('audio_features')
    try:
        audsrv = rospy.ServiceProxy('speech_to_text', Stt)
        transcript = audsrv(data.data)
        rospy.loginfo("audio feature extractor server successfully responded with %s",transcript)
        rospy.loginfo("Publishing audio features to /audio_features topic")
        pub.publish(transcript)
        
    except rospy.ServiceException as e:
        rospy.logerr("Service call failed: %s"%e)


if __name__ == '__main__':
    rospy.Subscriber('captured_audio', AudioData, callservice)
    rospy.spin()