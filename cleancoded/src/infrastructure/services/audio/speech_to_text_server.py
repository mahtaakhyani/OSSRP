#!/usr/bin/env python

'''
This module is the server for the speech-to-text service. It receives the audio data from the client and sends it to the Google Speech Recognition API. 
 Then, it receives the transcript and sends it to the client as a string.
'''

# pip install SpeechRecognition
# https://pypi.python.org/pypi/SpeechRecognition/
# recognizer_instance.recognize_google(audio_data, key = None, language = "fa-IR", show_all = False)
# Performs speech recognition on audio_data (an AudioData instance), using the Google Speech Recognition API.
# The Google Speech Recognition API key is specified by key. If not specified, it uses a generic key that works out of the box.
# This should generally be used for personal or testing purposes only, as it may be revoked by Google at any time.

import rospy
import speech_recognition as sr
from infrastructure.srv import Stt
from audio_common_msgs.msg import AudioDataStamped as AudioData

class SpeechToText:
	def __init__(self):
		rospy.init_node('speech_to_text_server')
		s = rospy.Service('speech_to_text', Stt, self.speech_to_text) 
		rospy.Subscriber("/captured_audio", AudioData, self.callback) 
		rospy.loginfo("Ready to convert speech to text.")

	def callback(self,data):
		self.audio = data.data

	def speech_to_text(self,req):
		r = sr.Recognizer() #initialize the recognizer
		r.operation_timeout = 30 #set the operation timeout to 30 seconds meaning that if the speech is not recognized within 30 seconds, the operation will be aborted

		# recognize speech using Google Speech Recognition
		try:
			rospy.log_info('Trying...')
			transcript = r.recognize_google(self.audio,language='fa-IR')
			# for testing purposes, we're just using the default API key
			# to use another API key, use `r.recognize_google(audio, key="GOOGLE_SPEECH_RECOGNITION_API_KEY")`
			# instead of `r.recognize_google(audio)`

			rospy.log_info("Google Speech Recognition thinks you said in persian: -  " + transcript)
			
			# Uncomment the following lines to write the transcript to a file
			# with open("transcript.txt","w+", encoding="utf-8") as tr:
			# 	tr.write(transcript)
			# 	tr.close()
		
			return transcript

		except sr.UnknownValueError:
			rospy.logerror("Google Speech Recognition could not understand audio") #log the error
			return('') #return empty string if the speech is not recognized

		except sr.RequestError as e:
			rospy.logerror("Could not request results from Google Speech Recognition service; {0}".format(e)) #log the error
			return('') #return empty string if the speech is not recognized
			


if __name__ == "__main__":
        stt = SpeechToText() 
        rospy.spin()