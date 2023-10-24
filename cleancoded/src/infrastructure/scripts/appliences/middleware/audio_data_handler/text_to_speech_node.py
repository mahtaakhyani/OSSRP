# This node is responsible for calling the speech_to_text service and sending the audio data to it.
# the pioneer node that subscribes to the audio data topic and send the audio data to speech to text server and receives the transcript as a String.
# Then it initiates a topic to publish the received transcript to the topic /transcript as a String.
#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from infrastructure.srv import Tts
from audio_common_msgs.msg import AudioData



rospy.init_node('text_to_speech_node',anonymous=False)
pub = rospy.Publisher('tts_output', AudioData, queue_size=10)

def callservice(data):
    rospy.loginfo('Got a TTS command on /say_sth. Calling the service.')
    rospy.wait_for_service('text_to_speech')
    try:
        audsrv = rospy.ServiceProxy('text_to_speech', Tts)
        audio = audsrv(data.data)
        print(type(audio.speech))
        rospy.loginfo("TTS server successfully responded")
        result_audio = AudioData()
        result_audio.data = audio.speech.data
        rospy.loginfo("Publishing generated audio to /tts_output topic")
        pub.publish(result_audio)
        
    except rospy.ServiceException as e:
        rospy.logerr("Service call failed: %s"%e)


if __name__ == '__main__':
    rospy.Subscriber('say_sth', String, callservice)
    rospy.spin()