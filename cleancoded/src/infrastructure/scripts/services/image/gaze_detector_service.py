
# This module is the server for the gaze detection service. 
# First, it subscribes to the extracted landmarkes from image frame topic and sends it to the gaze detection function.
# The main class of the service is GazeDetectorService which is a ROS service and it inherits from GazePosition class.
# It receives the image data from the client and sends it to the gaze detection function.
# The output is the gaze position as a string through the Gaze service [which is located in "$(rospack find infrastructure)/srv"].

#!/usr/bin/env python
import importlib.util
import math
import os
import sys

import cv2
import numpy as np
import rospkg
import rospy
from infrastructure.msg import List
from infrastructure.srv import Gaze
from sensor_msgs.msg import Image

pkg = rospkg.RosPack().get_path('infrastructure')
module_path = os.path.join(pkg, 'scripts', 'tools', 'helper_modules')
sys.path.append(module_path)


from mathhelpers import relative, relativeT, rel, normalize_points


# //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
class GazeTracking:
    print('in the GazeTrack')

    right_pupil = ''
    left_pupil = ''
    Eye_ball_center_left = ''
    Eye_ball_center_right = ''

    translation_vector = ''
    rotation_vector = ''
  
    @classmethod
    def gazedirection(self, data):
        self.points = data.landmark.face
        print('1')
        points_list = list(data.landmark.face)
        points = [(tuple([float(j) for j in i.replace('(','').replace(')','').replace(' ','').split(',')])) for i in points_list]
        fr = [list(i.data) for i in data.frame.list]
        frame = np.array(fr, dtype="float32")
        
        # The gaze function gets an image and face landmarks from mediapipe framework.
        # The function draws the gaze direction into the frame.

        
        # 2D image points.
        # relative takes mediapipe points that is normalized to [-1, 1] and returns image points
        # at (x,y) format
        print((points[4], 480,640))

        image_points = np.array([
            relative(points[4], [480,640]),  # Nose tip
            relative(points[152], [480,640]),  # Chin
            # Left eye left corner
            relative(points[263], [480,640]),
            # Right eye right corner
            relative(points[33], [480,640]),
            relative(points[287], [480,640]),  # Left Mouth corner
            relative(points[57], [480,640])  # Right mouth corner
        ], dtype="double")
        print('image_points done')

        # image_points = np.array([points[4],points[152],points[263],points[33],points[287],points[57]],dtype='float64')
        # image_points1 = np.array(rel([points[4],points[152],points[263],points[33],points[287],points[57]]),dtype='float64')

       
        # 2D image points.
        # relativeT takes mediapipe self.points that is normalized to [-1, 1] and returns image self.points
        # at (x,y,0) format
       
        image_points1 = np.array([
            relativeT(points[4]),  # Nose tip
            relativeT(points[152]),  # Chin
            # Left eye, left corner
            relativeT(points[263]),
            # Right eye, right corner
            relativeT(points[33]),
            relativeT(points[287]),  # Left Mouth corner
            relativeT(points[57])  # Right mouth corner
        ], dtype="double")

        print('image_points1 done.', rel(points[4]) )

        # 3D model points.
        model_points = np.array([
            (0.0, 0.0, 0.0),  # Nose tip
            (0, -63.6, -12.5),  # Chin
            (-43.3, 32.7, -26),  # Left eye, left corner
            (43.3, 32.7, -26),  # Right eye, right corner
            (-28.9, -28.9, -24.1),  # Left Mouth corner
            (28.9, -28.9, -24.1)  # Right mouth corner
        ])

        # 3D model eye self.points
        # The center of the eye ball
        
        self.Eye_ball_center_right = np.array([[-29.05], [32.7], [-39.5]])
        # the center of the left eyeball as a vector.
        self.Eye_ball_center_left = np.array([[29.05], [32.7], [-39.5]])

        print('eye balls done')

        
        # camera matrix estimation
        
        focal_length = frame.shape[1]
        center = (frame.shape[1] / 2, frame.shape[0] / 2)
        camera_matrix = np.array(
            [[focal_length, 0, center[0]],
             [0, focal_length, center[1]],
             [0, 0, 1]], dtype="double"
        )
        # Draw a circle on the frame center
        # cv2.circle(frame, tuple([int(i) for i in center]), 5, 255,0,0)

        dist_coeffs = np.zeros((4, 1))  # Assuming no lens distortion
        (success, self.rotation_vector, self.translation_vector) = cv2.solvePnP(model_points, image_points, camera_matrix,
                                                                                dist_coeffs, flags=cv2.SOLVEPNP_ITERATIVE)
        # 2d pupil location
        # self.left_pupil = relative(points[468], frame.shape)
        # self.right_pupil = relative(points[473], frame.shape)
        self.left_pupil = points[468]
        self.right_pupil = points[473]
        print('puppils location done')
        
        # Transformation between image point to world point
        _, transformation, _ = cv2.estimateAffine3D(
            image_points1, model_points)  # image to world transformation
        print(transformation, image_points1, model_points)
        if transformation is not None:  # if estimateAffine3D secsseded
            # project pupil image point into 3d world point
            pupil_world_cord = transformation @ np.array(
                [[self.left_pupil[0], self.left_pupil[1], 0, 1]]).T

            # 3D gaze point (10 is arbitrary value denoting gaze distance)
            S = self.Eye_ball_center_left + \
                (pupil_world_cord - self.Eye_ball_center_left) * 30

            # Project a 3D gaze direction onto the image plane.
            (eye_pupil2D, _) = cv2.projectPoints((int(S[0]), int(S[1]), int(S[2])), self.rotation_vector,
                                                 self.translation_vector, camera_matrix, dist_coeffs)
            # project 3D head pose into the image plane
            (head_pose, _) = cv2.projectPoints((int(pupil_world_cord[0]), int(pupil_world_cord[1]), int(40)),
                                               self.rotation_vector,
                                               self.translation_vector, camera_matrix, dist_coeffs)
            # correct gaze for head rotation
            gaze = self.left_pupil + \
                (eye_pupil2D[0][0] - self.left_pupil) - \
                (head_pose[0][0] - self.left_pupil)

            # Draw gaze line into screen
            p1 = (int(self.left_pupil[0]), int(self.left_pupil[1]))
            self.p1 = p1
            p2 = (int(gaze[0]), int(gaze[1]))
            cv2.line(frame, p1, p2, (0, 0, 255), 1)
            cv2.circle(frame, p1, 4, (0, 255, 0))
            cv2.circle(frame, p2, 8, (0, 255, 0))
    # ///////////////////////////////////////////////////////////////////////

            pupilr_world_cord = transformation @ np.array(
                [[self.right_pupil[0], self.right_pupil[1], 0, 1]]).T

            # 3D gaze point (10 is arbitrary value denoting gaze distance)
            Sr = self.Eye_ball_center_right + \
                (pupilr_world_cord - self.Eye_ball_center_right) * 30

            # Project a 3D gaze direction onto the image plane.
            (eye_pupilr2D, _) = cv2.projectPoints((int(Sr[0]), int(Sr[1]), int(Sr[2])), self.rotation_vector,
                                                  self.translation_vector, camera_matrix, dist_coeffs)
            # project 3D head pose into the image plane
            (head_pose, _) = cv2.projectPoints((int(pupilr_world_cord[0]), int(pupilr_world_cord[1]), int(40)),
                                               self.rotation_vector,
                                               self.translation_vector, camera_matrix, dist_coeffs)
            # correct gaze for head rotation
            self.gaze = self.right_pupil + \
                (eye_pupilr2D[0][0] - self.right_pupil) - \
                (head_pose[0][0] - self.right_pupil)

            # Draw gaze line into screen
            pr1 = (int(self.right_pupil[0]), int(self.right_pupil[1]))
            self.pr1 = pr1
            pr2 = (int(self.gaze[0]), int(self.gaze[1]))
            cv2.line(frame, pr1, pr2, (0, 0, 255), 1)
            cv2.circle(frame, pr1, 4, (0, 255, 0))
            cv2.circle(frame, pr2, 8, (0, 255, 0))
            # print((relativeT(self.points[263], frame.shape))[:2])
            # Draw circle on 3D projected head pose
            # cv2.circle(frame, (int(head_pose[0][0][0]),int(head_pose[0][0][1])), 14, (0,255,0))
            self.left_eye_left_corner = (
                rel(points[263]))[:2]
            self.left_eye_right_corner = (
                rel(points[398],))[:2]
            self.left_eye_top_corner = (
                rel(points[374]))[:2]
            self.left_eye_bottom_corner = (
                rel(points[386]))[:2]

    def feedback(self):
        try:
            return (self.p1, self.pr1,
                    self.Eye_ball_center_left,
                    self.Eye_ball_center_right,
                    self.left_eye_left_corner,
                    self.left_eye_right_corner,
                    self.left_eye_top_corner,
                    self.left_eye_bottom_corner)
        except:
            pass


class HeadPosition(GazeTracking):
    print('in the HeadPosition')

    def __init__(self, data):
        print('2')
        gp = super().gazedirection(data)
        self.rotation_vector = gp.rotation_vector
        self.translation_vector = gp.translation_vector

        font = cv2.FONT_HERSHEY_SIMPLEX
        # 3D model points.
        model_points = np.array([
            (0.0, 0.0, 0.0),             # Nose tip
            (0.0, -330.0, -65.0),        # Chin
            # Left eye left corner
            (-225.0, 170.0, -135.0),
            # Right eye right corne
            (225.0, 170.0, -135.0),
            # Left Mouth corner
            (-150.0, -150.0, -125.0),
            # Right mouth corner
            (150.0, -150.0, -125.0)
        ])

        image_points = np.array([
            relativeT(points[4], img.shape),  # Nose tip
            relativeT(points[152], img.shape),  # Chin
            # Left eye, left corner
            relativeT(points[263], img.shape),
            # Right eye, right corner
            relativeT(points[33], img.shape),
            relativeT(points[287], img.shape),  # Left Mouth corner
            relativeT(points.list[57], img.shape)  # Right mouth corner
        ], dtype="double")
        # Camera internals
        focal_length = img.shape[1]
        center = (img.shape[1]/2, img.shape[0]/2)
        camera_matrix = np.array(
            [[focal_length, 0, center[0]],
             [0, focal_length, center[1]],
             [0, 0, 1]], dtype="double"
        )

        dist_coeffs = np.zeros((4, 1))  # Assuming no lens distortion
        # Project a 3D point (0, 0, 1000.0) onto the image plane.
        # We use this to draw a line sticking out of the nose

        (nose_end_point2D, jacobian) = cv2.projectPoints(np.array(
            [(0.0, 0.0, 1000.0)]), self.rotation_vector, self.translation_vector, camera_matrix, dist_coeffs)

        p1 = (int(image_points[0][0]), int(image_points[0][1]))
        p2 = (int(nose_end_point2D[0][0][0]), int(nose_end_point2D[0][0][1]))
        x1, x2 = self.head_pose_points(img, camera_matrix)
        self.draw_annotation_box(img, camera_matrix)

        cv2.line(img, p1, p2, (0, 255, 255), 2)
        cv2.line(img, tuple(x1), tuple(x2), (255, 255, 0), 2)
        # for (x, y) in marks:
        #     cv2.circle(img, (x, y), 4, (255, 255, 0), -1)
        # cv2.putText(img, str(p1), p1, font, 1, (0, 255, 255), 1)
        try:
            m = (p2[1] - p1[1])/(p2[0] - p1[0])
            self.ang1 = int(math.degrees(math.atan(m)))
        except:
            self.ang1 = 90

        try:
            m = (x2[1] - x1[1])/(x2[0] - x1[0])
            self.ang2 = int(math.degrees(math.atan(-1/m)))
        except:
            self.ang2 = 90

        cv2.putText(img, str(self.ang1), tuple(
            p1), font, 2, (128, 255, 255), 3)
        cv2.putText(img, str(self.ang2), tuple(
            x1), font, 2, (255, 255, 128), 3)

        # print('div by zero error')
    def get_2d_points(self, img, camera_matrix, val):
        """Return the 3D points present as 2D for making annotation box"""
        point_3d = []
        dist_coeffs = np.zeros((4, 1))
        rear_size = val[0]
        rear_depth = val[1]
        point_3d.append((-rear_size, -rear_size, rear_depth))
        point_3d.append((-rear_size, rear_size, rear_depth))
        point_3d.append((rear_size, rear_size, rear_depth))
        point_3d.append((rear_size, -rear_size, rear_depth))
        point_3d.append((-rear_size, -rear_size, rear_depth))

        front_size = val[2]
        front_depth = val[3]
        point_3d.append((-front_size, -front_size, front_depth))
        point_3d.append((-front_size, front_size, front_depth))
        point_3d.append((front_size, front_size, front_depth))
        point_3d.append((front_size, -front_size, front_depth))
        point_3d.append((-front_size, -front_size, front_depth))
        point_3d = np.array(point_3d, dtype=float).reshape(-1, 3)

        # Map to 2d img points
        (point_2d, _) = cv2.projectPoints(point_3d,
                                          self.rotation_vector,
                                          self.translation_vector,
                                          camera_matrix,
                                          dist_coeffs) # returns 2d points as (x,y)
        point_2d = list(map(np.int32,point_2d.reshape(-1, 2))) # take in the x and y values of the 2d points and reshape them into a 2d array of points (x,y) for each point. the -1 means that the number of rows is inferred from the data provided,
        # and the 2 means that there are 2 columns (x,y) for each point. the map function takes in the np.int32() function and applies it to each element in the array so that the points are integers instead of floats (which is what the projectPoints function returns) because we need integers to draw the box around the face.
        # the output of this function is a list of tuples of the 2d points (x,y) for each point in the box around the face.
        return point_2d

    def draw_annotation_box(self, img, camera_matrix,
                            rear_size=300, rear_depth=0, front_size=500, front_depth=400,
                            color=(255, 255, 0), line_width=2):

        rear_size = 1
        rear_depth = 0
        front_size = img.shape[1]
        front_depth = front_size*2
        val = [rear_size, rear_depth, front_size, front_depth]
        point_2d = self.get_2d_points(img, camera_matrix, val)
        # # Draw all the lines
        cv2.polylines(img, [point_2d], True, color, line_width, cv2.LINE_AA) # draw the box around the face
        cv2.line(img, tuple(point_2d[1]), tuple(
            point_2d[6]), color, line_width, cv2.LINE_AA) # draw line from left ear to right ear
        cv2.line(img, tuple(point_2d[2]), tuple(
            point_2d[7]), color, line_width, cv2.LINE_AA) # draw line from left eye to right eye
        cv2.line(img, tuple(point_2d[3]), tuple(
            point_2d[8]), color, line_width, cv2.LINE_AA) # draw line from left mouth to right mouth

    def head_pose_points(self, img, camera_matrix):
        """
        Get the points to estimate head pose sideways    

        Parameters
        ----------
        img : np.unit8
            Original Image.
        rotation_vector : Array of float64
            Rotation Vector obtained from cv2.solvePnP
        translation_vector : Array of float64
            Translation Vector obtained from cv2.solvePnP
        camera_matrix : Array of float64
            The camera matrix

        Returns
        -------
        (x, y) : tuple
            Coordinates of line to estimate head pose

        """
        rear_size = 1
        rear_depth = 0
        front_size = img.shape[1]
        front_depth = front_size*2
        val = [rear_size, rear_depth, front_size, front_depth]
        point_2d = self.get_2d_points(img, camera_matrix, val)
        y = (point_2d[5] + point_2d[8])//2
        x = point_2d[2]

        return (x, y)

    def head_rpy(self):
        '''Returns a list containing the head pose as a dictionary of roll, pitch, yaw angles, 
            and a string of the head pose direction'''
        head_rpy_dict = {'pitch': self.ang1, 'yaw': self.ang2}
        head_pos_str = ['Head:']
        if self.ang1 in range(-40, 40) and self.ang2 in range(-40, 40):
            head_pos_str.append('center')
        else:
            if self.ang1 >= 30:
                head_pos_str.append('down')
                # cv2.putText(img, 'Head down', (30, 30), font, 2, (255, 255, 128), 3)
            elif self.ang1 <= -30:
                head_pos_str.append('up')
                # cv2.putText(img, 'Head up', (30, 30), font, 2, (255, 255, 128), 3)

            if self.ang2 >= 40:
                head_pos_str.append('right')
                # cv2.putText(img, 'Head right', (90, 90), font, 2, (255, 255, 128), 3)
            elif self.ang2 <= -40:
                head_pos_str.append('left')
                # cv2.putText(img, 'Head left', (90, 90), font, 2, (255, 255, 128), 3)

        str_pose = ' '.join(head_pos_str)
        all_data = [str_pose, head_rpy_dict]
        return (all_data)

    def __str__(self) -> str:
        return str(self.head_rpy()[0][5:])

# //////////////////////////////////////////////////////////////////////////////

class GazePosition(HeadPosition):
    print('in the GazePosition')
    def __init__(self, data):
        frame = data.frame
        print('3')
        super().__init__(data)
        print('head position super done')
        g = GazeTracking().feedback()
        self.left_center = g[0]
        self.right_center = g[1]
        self.left_corner = g[4]
        self.right_corner = g[5]
        self.top_corner = g[6]
        self.bottom_corner = g[7]
        self.frame = frame
        self.pupil_dir = "center"
        self.pupil_dir_y = " center"
        try:
            cv2.circle(self.frame, self.left_corner, 1, (0, 255, 0))
            cv2.circle(self.frame, self.right_corner, 1, (0, 255, 0))
            cv2.circle(self.frame, self.top_corner, 1, (0, 255, 0))
            cv2.circle(self.frame, self.bottom_corner, 1, (0, 255, 0))
        except:
            pass

        self.eye_width = tuple(
            i-j for i, j in zip(self.left_corner, self.right_corner))[0]
        self.eye_height = tuple(
            i-j for i, j in zip(self.top_corner, self.bottom_corner))[1]
        # cv2.circle(self.frame,self.right_center , self.eye_height, (0,255,0))

    def __str__(self):
        # isolate only the direction of the head (up,down,etc.)
        hp = super().__str__()
        detobj = self.dir_determin(hp)
  

        return detobj # returning the direction of the gaze

    def dir_determin(self, hp):
        if bool((math.fabs(self.right_center[0]-self.right_corner[0]) - self.eye_width/2) < -1):
            self.pupil_dir = "right"  # """Returns true if the user is looking to the right"""
        elif bool((math.fabs(self.right_center[0]-self.right_corner[0]) - self.eye_width/2) > 1):
            self.pupil_dir = 'left'  # """Returns true if the user is looking to the left"""


        if bool(math.fabs(math.fabs(self.right_center[1]-self.bottom_corner[1]) - self.eye_height/2) >= 1):
            self.pupil_dir_y = " up"  # """Returns true if the user is looking to the right"""
        else:
            self.pupil_dir_y = ' down'  # """Returns true if the user is looking to the left"""

        if self.pupil_dir in hp and self.pupil_dir_y in hp:
            return [self.pupil_dir+self.pupil_dir_y]

        else:
            return [self.pupil_dir+self.pupil_dir_y,' with head being ',hp]

# //////////////////////////////////////////////////////////////////////////////
#*******************************************************************************
# //////////////////////////////////////////////////////////////////////////////

class GazeDetectorService():
    rospy.init_node('gaze_detector', anonymous=False)
    def __init__(self):
        rospy.Service('gaze_pose', Gaze, self.callback)
        rospy.loginfo("gaze detector service successfully initiated")

    def callback(self, data):
        print('callback check ---------------------------------------------------------------------')
        try:
            frame = data.frame
            superclass = GazePosition(data)
            gazedir = superclass.__str__()
            rospy.loginfo("returning the gaze direction to the client")
            return gazedir

        except BaseException as bex:
            rospy.loginfo(rospy.ServiceException(bex))




# //////////////////////////////////////////////////////////////////////////////

if __name__ == '__main__':
    try:
        print('hey')
        GazeDetectorService()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
