#!/usr/bin/env python
# This node is responsible for calling the speech_to_text service and sending the audio data to it.
# the pioneer node that subscribes to the audio data topic and send the audio data to speech to text server and receives the transcript as a String.
# Then it initiates a topic to publish the received transcript to the topic /transcript as a String.

import rospy
from std_msgs.msg import String
from infrastructure.srv import Tts
from audio_common_msgs.msg import AudioData
import time



rospy.init_node('text_to_speech_node',anonymous=False)
pub = rospy.Publisher('audio/audio', AudioData, queue_size=10)

def callservice(data):
    rospy.loginfo('Got a TTS command on /say_sth. Calling the service.')
    rospy.wait_for_service('text_to_speech')
    try:
        audsrv = rospy.ServiceProxy('text_to_speech', Tts)
        audio = audsrv(data.data)
        print(type(audio.speech.data))
        rospy.loginfo("TTS server successfully responded")
        for chunk in audio.speech.data:
            pub.publish(chunk)
            time.sleep(0.1)
        rospy.loginfo("Publishing generated audio to /audio/audio topic")
        
    except rospy.ServiceException as e:
        rospy.logerr("Service call failed: %s"%e)


if __name__ == '__main__':
    rospy.Subscriber('say_sth', String, callservice)
    rospy.spin()