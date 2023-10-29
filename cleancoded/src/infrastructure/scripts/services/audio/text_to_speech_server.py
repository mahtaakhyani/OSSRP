#!/usr/bin/env python3.8

"""
This module is the server for the text-to-speech service. 
It subscribes to the topic /text_to_speech/text and publishes the audio data to the topic /text_to_speech/audio_data.
    (In other words, The web interface publishes the text to the topic /text_to_speech/text 
        and the robot plays the audio data from the topic /text_to_speech/audio_data)
"""
# from hazmtest import FarsGPT2
import asyncio
import os
import random
import sys
import edge_tts as edge_tts
from edge_tts import VoicesManager
import rospy
from std_msgs.msg import String
from infrastructure.srv import Tts
from audio_common_msgs.msg import AudioData


class TextToSpeechServer:
    outputfile = "hazm.wav"
    voice_name = "Microsoft Server Speech Text to Speech Voice (fa-IR, DilaraNeural)"

    def __init__(self):
        rospy.init_node('text_to_speech_server', anonymous=False)
        s = rospy.Service('text_to_speech', Tts, self.callback)
        rospy.loginfo("Ready to convert text to speech.")

    def callback(self, data):
        rospy.loginfo('Recieved the service call. Processing...')
        text = data.text
        audio = asyncio.run(self.text_to_speech(text))
        result_msg = AudioData()
        result_msg.data = audio
        # print(type(audio))
        rospy.loginfo('Processed the data. Sending back the results.')
        return result_msg

    async def text_to_speech(self, text):
        voices = await VoicesManager.create()
        # voice = voices.find(Gender="Female", Language="fa")
        voice = voices.find(Name=self.voice_name)
        rospy.loginfo(voice)
        # Also supports Locales
        # voice = voices.find(Gender="Female", Locale="es-AR") #Automated language-based voice selection

        communicate = edge_tts.Communicate(
            text, random.choice(voice)["Name"])
        # communicate = edge_tts.Communicate(TEXT, voice)
        await communicate.save(self.outputfile) # save to file
        
        
        async for message in communicate.stream(): # check the type of the message, if it is audio, return it, else log an error. 
            # The output of the stream function is a generator object. which basically means that we can access the audio data as a list of bytes. 
            # The returned message from the server in the stream function (original Edge-tts function in "communicate.py") includes the type of the message and the data. 
            # The type of the message can be either audio or error. if it is audio, the data is the audio data, else it is the error message.
                if message["type"] == "audio":
                    return message["data"]
                else:
                    rospy.logerr("Error in Microsoft Edge text to speech service") 
                    return []


if __name__ == "__main__":
    try:
        tts = TextToSpeechServer()
        rospy.spin()
    except rospy.ROSInterruptException as e:
        rospy.logdebug(e)



# fagpt = FarsGPT2('gpt2_fa')
# inputs = "ینی واقعا شدنیه که"
# output = fagpt.query({
#     "inputs": inputs,
# })
# print(output[0]['generated_text'])

# TEXT = str(output[0]['generated_text']).replace(inputs+'))). --', '')
# print(TEXT)