# This module is used to test the landmark module(a.k.a. pose.py)

import unittest
import sys
import os
import numpy as np
from sensor_msgs.msg import Image

# add the path to the cleancoded/src/infrastructure/scripts directory
sys.path.append(os.path.join(os.path.dirname(__file__), '..', '..', '..', '..', 'src', 'infrastructure', 'scripts'))

from basics.image.landmarks_detection.pose import MeshDetector

class TestPose(unittest.TestCase):
        
        def test_syncinfo(self): # test the syncinfo function in the MeshDetector class
            md = MeshDetector()
            md.syncinfo()
            self.assertIsNotNone(md)

        def test_catch(self): # test the catch function in the MeshDetector class
            md = MeshDetector()
            md.catch()
            self.assertEqual(type(md.catch()), list) # check if the type of the output is list, else it would be error

        def test_convert_back(self): # test the convert_back function in the MeshDetector class
            md = MeshDetector()
            md.convert_back()
            self.assertEqual(type(md.convert_back()), Image) # check if the type of the output is Image, else it would be error
        
        def test_analyze(self): # test the analyze function in the MeshDetector class
            md = MeshDetector()
            md.analyze()
            self.assertEqual(type(md.analyze()), list) # check if the type of the output is list, else it would be error


    
        def test_meshdetector(self): # test the MeshDetector class
            md = MeshDetector()
            md.pointing()
            self.assertIsNotNone(md) # check if the output is not None, else it would be error
    
        def test_draw(self): # test the draw function in the MeshDetector class
            md = MeshDetector()
            md.draw()
            self.assertEqual(type(md.draw()), np.ndarray) # check if the type of the output is numpy array, else it would be error

        

if __name__ == '__main__':
    import rosunit
    rosunit.unitrun('tests', 'TestPose', TestPose)
    rospy.spin()
