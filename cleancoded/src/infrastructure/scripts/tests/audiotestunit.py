# This module is used to test the audio module.

import unittest
import sys
import os
import numpy as np
import rospy

# add the path to the cleancoded/src/infrastructure/scripts directory
sys.path.append(os.path.join(os.path.dirname(__file__), '..', '..', '..', '..', 'src', 'infrastructure', 'scripts'))

from basics.audio.AudioCapture import AudioCapture

class TestAudioCapture(unittest.TestCase):
    def test_capture(self): # test the capture function in the AudioCapture class
        aud = AudioCapture()
        aud.capture()
        self.assertIsNotNone(aud) # check if the output is not None, else it would be error

    def test_stop(self): # test the stop function in the AudioCapture class
        aud = AudioCapture()
        aud.stop()
        self.assertIsNotNone(aud) # check if the output is not None, else it would be error

if __name__ == '__main__':
    import rosunit
    rosunit.unitrun('tests', 'TestAudioCapture', TestAudioCapture)
    rospy.spin()