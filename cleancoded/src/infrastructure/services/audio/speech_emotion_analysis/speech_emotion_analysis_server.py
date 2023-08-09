'''
This module is the server for the speech emotion analysis service. 
It receives the audio data from the client and sends it to the feature extraction service. 
Then, it receives the extracted features in a custom message called AudFeature [which is located in "$(rospack find infrastructure)/msg"] and sends them as a list of floats to the emotion classification module to predict the emotion of the ongoing speech. (emotion_classification.py module is a python tool, not a ROS node. It is located in "$(rospack find infrastructure)/tools"**).

Then, it receives the emotion of the ongoing speech as a string from the emotion classification module and sends it back to the client via the EmoProb service [which is located in "$(rospack find infrastructure)/srv"].

**[It's a machine learning model that can be trained to predict the emotion of the ongoing speech based on the extracted features of the speech. The dataset is located in "$(rospack find infrastructure)/tools/datasets")]
'''

#!/usr/bin/env python
import rospy
from infrastructure.srv import EmoProb, AudFeature
from audio_common_msgs.msg import AudioDataStamped as AudioData
from emotion_classification import Classification

class SpeechEmotionAnalysis:
    rospy.init_node('speech_emotion_analysis_server')
    
    def __init__(self):
        rospy.Service('speech_emotion_analysis', EmoProb, self.callback) 
    
    def callback(self,data):
        rospy.wait_for_service('audio_features')
        try:
            audfeaturesrv = rospy.ServiceProxy('audio_features', AudFeature)
            features = [audfeaturesrv(data)]
            emotion = Classification.prediction(item=features)
            return emotion
            
    
        except rospy.ServiceException as e:
            rospy.logerror("Service call failed: %s"%e)



if __name__ == "__main__":
        stt = SpeechEmotionAnalysis() 
        rospy.spin() #keeps python from exiting until this node is stopped