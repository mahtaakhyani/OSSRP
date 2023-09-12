# unit test for testing the camera module

import unittest
import sys
import os
import numpy as np



# add the path to the cleancoded/src/infrastructure/scripts directory
sys.path.append(os.path.join(os.path.dirname(__file__), '..', '..', '..', '..', 'src', 'infrastructure', 'scripts'))

from basics.image.CameraCapture import CameraCapture

class TestCameraCapture(unittest.TestCase):
    def test_convert_frame(self): # test the convert_frame function in the CameraCapture class
        cam = CameraCapture()
        cam.convert_frame()
        self.assertEqual(type(cam.modif_image), np.ndarray) # check if the type of the output is numpy array, else it would be error
    
    def test_syncinfo(self): # test the syncinfo function in the CameraCapture class
        cam = CameraCapture()
        cam.syncinfo()
        self.assertIsNotNone(cam) # check if the output is not None, else it would be error

if __name__ == '__main__':
    import rosunit
    rosunit.unitrun('tests', 'TestCameraCapture', TestCameraCapture)
    rospy.spin()