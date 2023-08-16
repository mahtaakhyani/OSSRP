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
from .tools.emotion_classification import Classification


class SpeechEmotionAnalysis:
    # Initialize the node and create a service
    rospy.init_node('speech_emotion_analysis_server')

    def __init__(self):
        rospy.Service('speech_emotion_analysis', EmoProb, self.callback)
        # Create a service named speech_emotion_analysis with the EmoProb service type [which is located in "$(rospack find infrastructure)/srv"]
        # The EmoProb service type has two fields:
        # 1. request: audio data (AudioDataStamped message type)
        # 2. response: emotion of the ongoing speech (string)

    def callback(self, data):
        # Wait for the audio_features service to be available
        rospy.wait_for_service('audio_features')
        try:
            # Create a proxy for the audio_features service to call it
            audfeaturesrv = rospy.ServiceProxy('audio_features', AudFeature)
            # Call the audio_features service and get the extracted features of the speech
            features = [audfeaturesrv(data)]

            # Predict the emotion of the ongoing speech based on the extracted features of the speech and return it as a string
            emotion = Classification.prediction(item=features)
            # The classification module is a python tool, not a ROS node. It is located in "$(rospack find infrastructure)/tools" and
            # uses the machine learning model that is trained to predict the emotion of the ongoing speech based on the extracted features of the speech.
            # The dataset is located in "$(rospack find infrastructure)/tools/datasets

        # ---------
        # Alternative: Sentiment analysis using Empath (text-based, faster, but less accurate):
            # import empath
            # from hazm import word_tokenize
            # from std_msgs.msg import String

            # # Get the text from the audio data
            # rospy.wait_for_service('speech_to_text')
            # try:
            #     sttsrv = rospy.ServiceProxy('speech_to_text', String)
            #     text = sttsrv(data)
            # except rospy.ServiceException as e:
            #     rospy.logerror("Service call failed: %s"%e)

            # # Create an Empath analyzer
            # ea = empath.Empath()
            # # Tokenize the text using hazm
            # tokens = word_tokenize(text)
            # # Analyze the emotions in the text using Empath
            # emotions = ea.analyze (' '.join(tokens), normalize=True)
        # ---------
            return emotion

        except rospy.ServiceException as E:
            rospy.logerror("Service call failed: %s" % E)


if __name__ == "__main__":
    try:
        stt = SpeechEmotionAnalysis()
        rospy.spin()  # keeps python from exiting until this node is stopped
    except rospy.ROSInterruptException:
        pass
