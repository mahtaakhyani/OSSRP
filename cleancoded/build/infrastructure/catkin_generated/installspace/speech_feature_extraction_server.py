#!/usr/bin/env python3
"""
This module is the server for the audio feature extraction service. 
 The output is the extracted features as a list of floats in a custom message called AudFeature [which is located in "$(rospack find infrastructure)/srv"]
 This service file takes the audio data as an input and returns the extracted features as an output.

That being said, this module receives the audio data from the client and sends it to the speech-to-text service.
 Then, it receives the transcript and sends it to the feature extraction function to extract the features of the ongoing speech. 
   
 The extracted features are:
    1. Minimum F0
    2. Maximum F0
    3. Mean F0
    4. Minimum Intensity
    5. Maximum Intensity
    6. Mean Intensity
    7. Jitter
    8. Shimmer
    9. HNR
    10. Speaking Rate

"""

import os
import sys
import rospy
import parselmouth
from parselmouth.praat import call
from infrastructure.srv import Stt, AudFeature



class FeatureExtractor:
    rospy.init_node('audio_feature_extractor',anonymous=False) 

    def __init__(self):
        rospy.Service('audio_features', AudFeature, self.callback) 
        
    def callback(self,data):
        audio = data.data
        num_words = self.transcript_handler(audio)
        features = self.feature_extractor(audio_data=audio, num_words=num_words)
        return features
        
    
    def transcript_handler(self,audio_data):
        rospy.wait_for_service('speech_to_text')
        try:
            sttsrv = rospy.ServiceProxy('speech_to_text', Stt)
            transcript = sttsrv(audio_data)
            num_words = len(transcript.split())
            return num_words
            
    
        except rospy.ServiceException as e:
            rospy.logerror("Service call failed: %s"%e)
            return(0.001)

    def feature_extractor(self,audio_data,num_words):
        rospy.loginfo("Extracting audio data's features...") #Source: Robot's microphone (not the client's)
        # extracts the duration
        duration = audio_data.get_total_duration()
        # extracts the pitch metrics
        pitch = call(audio_data, "To Pitch", 0.0, 75.0, 600.0)
        minF0 = call(pitch, "Get minimum", 0.0, duration, "Hertz", "Parabolic")
        maxF0 = call(pitch, "Get maximum", 0.0, duration, "Hertz", "Parabolic")
        avgF0 = call(pitch, "Get mean", 0.0, duration, "Hertz")
        # extracts the intensity metrics
        intensity = call(audio_data, "To Intensity", 75.0, 0.0)
        min_intensity = intensity.get_minimum()
        max_intensity = intensity.get_maximum()
        avg_intensity = intensity.get_average()
        # extracts jitter
        point_process = call(audio_data, "To PointProcess (periodic, cc)", 75.0, 600.0)
        jitter = call(point_process, "Get jitter (local)", 0.0, 0.0, 0.0001, 0.02, 1.3)
        # extracts shimmer
        shimmer = call(
            [audio_data, point_process],
            "Get shimmer (local)",
            0,
            0,
            0.0001,
            0.02,
            1.3,
            1.6,
        )
        # extracts HNR (harmonics-to-noise ratio)
        harmonicity = call(audio_data, "To Harmonicity (cc)", 0.01, 75.0, 0.1, 1.0)
        hnr = call(harmonicity, "Get mean", 0, 0)

        # extracts speaking rate
        speaking_rate = num_words / duration

    
        audio_features = AudFeature()
        audio_features.header.stamp = rospy.Time.now()
        audio_features.header.frame_id = "audio_features"

        audio_features.min_f0 , audio_features.max_f0 ,\
            audio_features.mean_f0 ,audio_features.min_int ,\
                audio_features.min_max ,audio_features.mean_int ,audio_features.jitter ,\
                    audio_features.shimmer ,audio_features.hnr ,\
                          audio_features.speaking_rate = map(lambda x: round(x, 3), 
                                                            [minF0,maxF0 ,avgF0 ,
                                                                    min_intensity , max_intensity ,
                                                                    avg_intensity ,jitter ,shimmer ,
                                                                    hnr ,speaking_rate]) 


        return audio_features

if __name__ == "__main__":
        featext = FeatureExtractor()
        rospy.spin()









    
    



def main(transcript=""):
    columns=["Emotion",
            "Min F0",
            "Max F0",
            "Mean F0",
            "Min Int",
            "Max Int",
            "Mean Int",
            "Jitter",
            "Shimmer",
            "HNR",
            "Speaking Rate"]


    input_sound = parselmouth.Sound(file)

    # extracts the duration
    duration = input_sound.get_total_duration()
    # extracts the pitch metrics
    pitch = call(input_sound, "To Pitch", 0.0, 75.0, 600.0)
    minF0 = call(pitch, "Get minimum", 0.0, duration, "Hertz", "Parabolic")
    maxF0 = call(pitch, "Get maximum", 0.0, duration, "Hertz", "Parabolic")
    avgF0 = call(pitch, "Get mean", 0.0, duration, "Hertz")
    # extracts the intensity metrics
    intensity = call(input_sound, "To Intensity", 75.0, 0.0)
    min_intensity = intensity.get_minimum()
    max_intensity = intensity.get_maximum()
    avg_intensity = intensity.get_average()
    # extracts jitter
    point_process = call(input_sound, "To PointProcess (periodic, cc)", 75.0, 600.0)
    jitter = call(point_process, "Get jitter (local)", 0.0, 0.0, 0.0001, 0.02, 1.3)
    # extracts shimmer
    shimmer = call(
        [input_sound, point_process],
        "Get shimmer (local)",
        0,
        0,
        0.0001,
        0.02,
        1.3,
        1.6,
    )
    # extracts HNR
    harmonicity = call(input_sound, "To Harmonicity (cc)", 0.01, 75.0, 0.1, 1.0)
    hnr = call(harmonicity, "Get mean", 0, 0)


    # extracts speaking rate
    transcript = get_transcript("transcript.txt", file_name)
    num_words = len(transcript.split())
    speaking_rate = num_words / duration

    # assembles the table
    metrics = [
        i,
        round(minF0, 3),
        round(maxF0, 3),
        round(avgF0, 3),
        round(min_intensity, 3),
        round(max_intensity, 3),
        round(avg_intensity, 3),
        round(jitter, 3),
        round(shimmer, 3),
        round(hnr, 3),
        round(speaking_rate, 3),
    ]


if __name__ == "__main__": 
    try:
        main()
    except rospy.ROSInterruptException:
        pass
    
