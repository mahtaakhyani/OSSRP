# This module is used to test the gaze module.

import unittest
import sys
import os
import numpy as np
import rospy

# add the path to the cleancoded/src/infrastructure/scripts directory
sys.path.append(os.path.join(os.path.dirname(__file__), '..', '..', '..', '..', 'src', 'infrastructure', 'scripts'))

from services.image.gaze_detector_service import GazeDetectorService, GazePosition, HeadPosition, GazeTracking

class TestGazeDetector(unittest.TestCase):

    def test_service_callback(self): # test the service_callback function in the GazeDetectorService class
        gaze = GazeDetectorService()
        gaze.service_callback()
        self.assertIsNotNone(gaze)


    
    def test_gaze_direction(self): # test the gaze_direction in GazeTracking class
        gaze = GazeTracking()
        gaze.gaze_direction()
        self.assertIsNotNone(gaze) # check if the output is not None, else it would be error

    def test_gaze_direction_feedback(self): # test the gaze_direction_feedback in GazeTracking class
        gaze = GazeTracking()
        gaze.feedback()
        self.assertEqual(type(gaze.feedback()), str) # check if the type of the output is string, else it would be error



    def test_get_2d_points(self): # test the get_2d_points in HeadPosition class
        head = HeadPosition()
        head.get_2d_points()
        self.assertEqual(type(head.get_2d_points()), list) # check if the type of the output is list, else it would be error
    
    def test_draw_annotation_box(self): # test the draw_annotation_box in HeadPosition class
        head = HeadPosition()
        head.draw_annotation_box()
        self.assertIsNone(head)
    
    def test_head_pose_points(self):
        head = HeadPosition()
        head.head_pose_points()
        self.assertEqual(type(head.head_pose_points()), tuple) # check if the type of the output is tuple, else it would be error

    def test_head_rpy(self):
        head = HeadPosition()
        head.head_rpy()
        self.assertEqual(type(head.head_rpy()), list) # check if the type of the output is list, else it would be error

    def test_str_HeadPosition(self):
        head = HeadPosition()
        head.__str__()
        self.assertEqual(type(head.__str__()), str)



    def test_dir_determine(self): # test the dir_determine in GazePosition class
        gaze = GazePosition()
        gaze.dir_determine()
        self.assertEqual(type(gaze.dir_determine()), list)

    def test_str_GazePosition(self):
        gaze = GazePosition()
        gaze.__str__()
        self.assertEqual(type(gaze.__str__()), str)

    

if __name__ == '__main__':
    import rosunit
    rosunit.unitrun('tests', 'TestGazeDetector', TestGazeDetector)
    rospy.spin()
