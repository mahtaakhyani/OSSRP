"""
This module is the server for the gaze detection service. 
It receives the image data from the client and sends it to the GazeDetector class to detect the gaze position through module gaze_detector.py.
The output is the gaze position as a string through the Gaze service [which is located in "$(rospack find infrastructure)/srv"].
"""
#!`/usr/bin/env python
import rospy
from sensor_msgs.msg import Image
from infrastructor.srv import Gaze
from cleancoded.src.infrastructure.services.image.GazeDetector.gaze_detector import GazePosition



class GazeDetectorService:
    rospy.init_node('gaze_detector', anonymous=False)
    
    def __init__(self):
        rospy.Service('gaze_pose', Gaze, self.callback)
    
    def callback(self, frame):
        try:
            g_pose = GazePosition(frame).__str__()
            return g_pose
        except BaseException as bex:
            return bex
        
if __name__ == '__main__':
    try:
        GazeDetectorService()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass