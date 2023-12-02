
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
from infrastructure.msg import List, Array3D
from infrastructure.srv import Gaze
from sensor_msgs.msg import Image, CameraInfo

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
	height = 480
	width = 640

  
	def gazedirection(self, data):
		self.points = data.landmark.face
		points = data.landmark.face
		arr = []
		l = self.convert_frame(data.frame).data
		for i in range(len(l)):
			arr.append(list(l[i].data))
		self.frame= np.array(arr, dtype=np.uint8).reshape((self.width, self.height,3))
		
		# The gaze function gets an image and face landmarks from mediapipe self.framework.
		# The function draws the gaze direction into the self.frame.

		
		# 2D image points.
		# relative takes mediapipe points that is normalized to [-1, 1] and returns image points
		# at (x,y) format

		self.image_points = np.array([
			relative(points[4], [self.height, self.width]),  # Nose tip
			relative(points[152], [self.height, self.width]),  # Chin
			# Left eye left corner
			relative(points[263], [self.height, self.width]),
			# Right eye right corner
			relative(points[33], [self.height, self.width]),
			relative(points[287], [self.height, self.width]),  # Left Mouth corner
			relative(points[57], [self.height, self.width])  # Right mouth corner
		], dtype="double")
		print('self.image_points done')

   
		# 2D image points.
		# relativeT takes mediapipe self.points that is normalized to [-1, 1] and returns image self.points
		# at (x,y,0) format
	   
		self.image_points1 = np.array([
			relativeT(points[4], [self.height, self.width]),  # Nose tip
			relativeT(points[152], [self.height, self.width]),  # Chin
			# Left eye, left corner
			relativeT(points[263], [self.height, self.width]),
			# Right eye, right corner
			relativeT(points[33], [self.height, self.width]),
			relativeT(points[287], [self.height, self.width]),  # Left Mouth corner
			relativeT(points[57], [self.height, self.width])  # Right mouth corner
		], dtype="double")

		print('self.image_points1 done.' )

		# 3D model points.
		self.model_points = np.array([
			(0.0, 0.0, 0.0),  # Nose tip
			(0, -63.6, -12.5),  # Chin
			(-43.3, 32.7, -26),  # Left eye, left corner
			(43.3, 32.7, -26),  # Right eye, right corner
			(-28.9, -28.9, -24.1),  # Left Mouth corner
			(28.9, -28.9, -24.1)  # Right mouth corner
		])

		# 3D model eye self.points
		# The self.center of the eye ball
		
		self.Eye_ball_center_right = np.array([[-29.05], [32.7], [-39.5]])
		# the self.center of the left eyeball as a vector.
		self.Eye_ball_center_left = np.array([[29.05], [32.7], [-39.5]])

		
		# camera matrix estimation
		self.focal_length = self.width
		self.center = (self.width / 2, self.height / 2)
		self.camera_matrix = np.array(
			[[self.focal_length, 0, self.center[0]],
			 [0, self.focal_length, self.center[1]],
			 [0, 0, 1]], dtype="double"
		)
		# Draw a circle on the self.frame self.center

		self.dist_coeffs = np.zeros((4, 1))  # Assuming no lens distortion
		(success, self.rotation_vector, self.translation_vector) = cv2.solvePnP(self.model_points, self.image_points, self.camera_matrix,
																		 self.dist_coeffs, flags=cv2.SOLVEPNP_ITERATIVE)

		# 2d pupil location
		self.left_pupil = relative(points[468], self.frame.shape)
		self.right_pupil = relative(points[473], self.frame.shape)
		cv2.circle(self.frame, tuple([int(i) for i in self.left_pupil]), 5, 255,0,0)
		print('left pupil',self.left_pupil)
		
		# Transformation between image point to world point
		_, transformation, _ = cv2.estimateAffine3D(
			self.image_points1, self.model_points)  # image to world transformation
		if transformation is not None:  # if estimateAffine3D secsseded
			# project pupil image point into 3d world point
			pupil_world_cord = transformation @ np.array(
				[[self.left_pupil[0], self.left_pupil[1], 0, 1]]).T
			# 3D gaze point (10 is arbitrary value denoting gaze distance)
			S = self.Eye_ball_center_left + \
				(pupil_world_cord - self.Eye_ball_center_left) * 30
			# Project a 3D gaze direction onto the image plane.
			(eye_pupil2D, _) = cv2.projectPoints((int(S[0]), int(S[1]), int(S[2])), self.rotation_vector,
												 self.translation_vector, self.camera_matrix, self.dist_coeffs)
			# project 3D head pose into the image plane
			(head_pose, _) = cv2.projectPoints((int(pupil_world_cord[0]), int(pupil_world_cord[1]), int(40)),
											   self.rotation_vector,
											 self.translation_vector, self.camera_matrix, self.dist_coeffs)
			# correct gaze for head rotation
			gaze = self.left_pupil + \
				(eye_pupil2D[0][0] - self.left_pupil) - \
				(head_pose[0][0] - self.left_pupil)
			# Draw gaze line into screen
			p1 = (int(self.left_pupil[0]), int(self.left_pupil[1]))
			self.p1 = p1
			p2 = (int(gaze[0]), int(gaze[1]))

			cv2.line(self.frame, p1, p2, (0, 0, 255), 1)
			cv2.circle(self.frame, p1, 4, (0, 255, 0))
			cv2.circle(self.frame, p2, 8, (0, 255, 0))
	# ///////////////////////////////////////////////////////////////////////

			pupilr_world_cord = transformation @ np.array(
				[[self.right_pupil[0], self.right_pupil[1], 0, 1]]).T

			# 3D gaze point (10 is arbitrary value denoting gaze distance)
			Sr = self.Eye_ball_center_right + \
				(pupilr_world_cord - self.Eye_ball_center_right) * 30

			# Project a 3D gaze direction onto the image plane.
			(eye_pupilr2D, _) = cv2.projectPoints((int(Sr[0]), int(Sr[1]), int(Sr[2])), self.rotation_vector,
												  self.translation_vector, self.camera_matrix, self.dist_coeffs)
			# project 3D head pose into the image plane
			(head_pose, _) = cv2.projectPoints((int(pupilr_world_cord[0]), int(pupilr_world_cord[1]), int(40)),
											   self.rotation_vector,
											   self.translation_vector, self.camera_matrix, self.dist_coeffs)
			# correct gaze for head rotation
			self.gaze = self.right_pupil + \
				(eye_pupilr2D[0][0] - self.right_pupil) - \
				(head_pose[0][0] - self.right_pupil)
			# Draw gaze line into screen
			pr1 = (int(self.right_pupil[0]), int(self.right_pupil[1]))
			self.pr1 = pr1
			pr2 = (int(self.gaze[0]), int(self.gaze[1]))
			self.pr2 = pr2
			cv2.line(self.frame, pr1, pr2, (0, 0, 255), 1)
			cv2.circle(self.frame, pr1, 4, (0, 255, 0))
			cv2.circle(self.frame, pr2, 8, (0, 255, 0))
			# Draw circle on 3D projected head pose
			cv2.circle(self.frame, (int(head_pose[0][0][0]),int(head_pose[0][0][1])), 14, (0,255,0))
			self.left_eye_left_corner = (
				relativeT(points[263], [self.height, self.width]))[:2]
			self.left_eye_right_corner = (
				relativeT(points[398], [self.height, self.width]))[:2]
			self.left_eye_top_corner = (
				relativeT(points[374], [self.height, self.width]))[:2]
			self.left_eye_bottom_corner = (
				relativeT(points[386], [self.height, self.width]))[:2]
		self.head_position()
		dict_data = self.head_rpy()[0][5:]
		direction = self.dir_determin(dict_data)

		return(''.join(direction))


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


	def head_position(self):
		(nose_end_point2D, jacobian) = cv2.projectPoints(np.array(
			[(0.0, 0.0, 1000.0)]), self.rotation_vector, self.translation_vector, self.camera_matrix, self.dist_coeffs)

		p1 = (int(self.image_points[0][0]), int(self.image_points[0][1]))
		p2 = (int(nose_end_point2D[0][0][0]), int(nose_end_point2D[0][0][1]))
		x1, x2 = self.head_pose_points(self.frame, self.camera_matrix)

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
		self.camera_matrix : Array of float64
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
		point_2d = self.get_2d_points(img, self.camera_matrix, val)
		y = (point_2d[5] + point_2d[8])//2
		x = point_2d[2]

		return (x, y)

	def get_2d_points(self, img, camera_matrix, val):
		"""Return the 3D points present as 2D for making annotation box"""
		point_3d = []
		self.dist_coeffs = np.zeros((4, 1))
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
										  self.camera_matrix,
										  self.dist_coeffs) # returns 2d points as (x,y)
		point_2d = list(map(np.int32,point_2d.reshape(-1, 2))) # take in the x and y values of the 2d points and reshape them into a 2d array of points (x,y) for each point. the -1 means that the number of rows is inferred from the data provided,
		# and the 2 means that there are 2 columns (x,y) for each point. the map function takes in the np.int32() function and applies it to each element in the array so that the points are integers instead of floats (which is what the projectPoints function returns) because we need integers to draw the box around the face.
		# the output of this function is a list of tuples of the 2d points (x,y) for each point in the box around the face.
		return point_2d

	def head_rpy(self):
		'''Returns a list containing the head pose as a dictionary of roll, pitch, yaw angles, 
			and a string of the head pose direction'''
		head_rpy_dict = {'pitch': self.ang1, 'yaw': self.ang2}
		print(head_rpy_dict)
		rospy.loginfo('Head pitch and yaw extracted successfully.')

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

    # ---- Results -----
	def dir_determin(self,hp):
		g = self.feedback()
		self.left_center = g[0]
		self.right_center = g[1]
		self.left_corner = g[4]
		self.right_corner = g[5]
		self.top_corner = g[6]
		self.bottom_corner = g[7]

		self.eye_width = tuple(
			i-j for i, j in zip(self.left_corner, self.right_corner))[0]
		self.eye_height = tuple(
			i-j for i, j in zip(self.top_corner, self.bottom_corner))[1]
		
		slip = (self.pr2[1]-self.pr1[1])/(self.pr2[0]-self.pr1[0])
		if bool((math.fabs(self.right_center[0]-self.right_corner[0]) - self.eye_width*2) < 0):
			self.pupil_dir = "right"  # """Returns true if the user is looking to the right"""
		if bool((math.fabs(self.right_center[0]-self.right_corner[0]) - self.eye_width*2) > 0):
			self.pupil_dir = 'left'  # """Returns true if the user is looking to the left"""

		# if bool(math.fabs(math.fabs(self.right_center[1]-self.right_corner[1]) - self.eye_height*2) > 0):
		if slip > -1:
			self.pupil_dir_y = " up"  # """Returns true if the user is looking to the right"""
		# if bool(math.fabs(math.fabs(self.right_center[1]-self.right_corner[1]) - self.eye_height*2) < 0):
		else:   
			self.pupil_dir_y = ' down'  # """Returns true if the user is looking to the left"""
		print(slip)
		gaze_rpy_dict = {'right eye corner': self.left_center, '\n eye height/2': self.eye_height/2}
		print(gaze_rpy_dict,'\n\n\n\n\n')
		self.convert_back(self.frame)
		if self.pupil_dir in hp and self.pupil_dir_y in hp:
			return [self.pupil_dir+self.pupil_dir_y]

		else:
			return [self.pupil_dir+self.pupil_dir_y,' with head being ',hp]
		
        
	

    # --- Frame manipulation ----
	def convert_back(self,cv2_img):
		pub = rospy.Publisher('/gaze_frame',Image,queue_size=10)
		ros_image = Image()
		ros_image.header.stamp = rospy.Time.now()
		ros_image.height = cv2_img.shape[0]
		ros_image.width = cv2_img.shape[1]
		ros_image.encoding = "bgr8"
		ros_image.step = cv2_img.shape[1] * 3
		ros_image.data = np.array(cv2_img).tobytes()		
		pub.publish(ros_image)
	
		# rospy.loginfo("image height: %s width: %s"%(self.height,self.width))
		try:
			return self.height, self.width
		except:
			pass	
	
	def convert_frame(self, data, cv2window=False): # convert the frame to a numpy array from ROS image
		try:
			encoding = data.encoding
			data = data.data
			# Convert image data to numpy array
			np_arr = np.frombuffer(data, np.uint8)
			cv2_img = np_arr.reshape((self.height, self.width, -1))

			# Convert image encoding if necessary
			if encoding != 'bgr8':
							cv2_img = cv2.cvtColor(cv2_img, cv2.COLOR_RGB2BGR)
			frame = cv2_img
			self.modif_image = cv2.resize(frame, (self.height, self.width)) # resize the image to the desired size which is the camera info size
			self.modif_image = cv2.cvtColor(self.modif_image, cv2.COLOR_BGR2RGB) # convert the image to RGB
			self.modif_image.flags.writeable =False # To improve performance, optionally mark the image as not writeable to pass by reference.

			tem = List()
			llll=self.modif_image.tolist()
			for i in llll:
				for j in i:
						obj=Array3D()
						obj.data=j
						tem.data.append(obj) # convert the image to a list of 3D arrays 
			rospy.loginfo("Converted the image to a list of 3D arrays")
			arr = []
			l = tem.data

			for i in range(len(l)):
							arr.append(l[i].data) # convert the list of 3D arrays to a list of lists since the 3D array is not supported in ROS
			
		
			return tem
						
		except Exception as e:
			rospy.logerr("Error: %s"%e)
			return e



# class HeadPosition(GazeTracking):
#	 print('in the HeadPosition')

#	 def __init__(self, data):
#		 print('2')
#		 gp = super().gazedirection(data)
#		 gp = GazeTracking()
#		 print('in the HeadPosition')
#		 self.rotation_vector = gp.rotation_vector
#		 self.translation_vector = gp.translation_vector
#		 print(self.rotation_vector)

#		 self.model_points = gp.model_points
#		 self.image_points = gp.image_points
#		 self.image_points1 = gp.image_points1

#		 self.focal_length = gp.focal_length
#		 self.center = gp.center
#		 self.camera_matrix = gp.camera_matrix

#		 img = gp.frame

#		 font = cv2.FONT_HERSHEY_SIMPLEX
#		 # 3D model points.
  
#		 self.dist_coeffs = np.zeros((4, 1))  # Assuming no lens distortion
#		 # Project a 3D point (0, 0, 1000.0) onto the image plane.
#		 # We use this to draw a line sticking out of the nose

#		 (nose_end_point2D, jacobian) = cv2.projectPoints(np.array(
#			 [(0.0, 0.0, 1000.0)]), self.rotation_vector, self.translation_vector, self.camera_matrix, self.dist_coeffs)

#		 p1 = (int(self.image_points[0][0]), int(self.image_points[0][1]))
#		 p2 = (int(nose_end_point2D[0][0][0]), int(nose_end_point2D[0][0][1]))
#		 x1, x2 = self.head_pose_points(img, self.camera_matrix)
#		 # self.draw_annotation_box(img, self.camera_matrix)

#		 # cv2.line(img, p1, p2, (0, 255, 255), 2)
#		 # cv2.line(img, tuple(x1), tuple(x2), (255, 255, 0), 2)
#		 # for (x, y) in marks:
#		 #	 cv2.circle(img, (x, y), 4, (255, 255, 0), -1)
#		 # cv2.putText(img, str(p1), p1, font, 1, (0, 255, 255), 1)
#		 try:
#			 m = (p2[1] - p1[1])/(p2[0] - p1[0])
#			 self.ang1 = int(math.degrees(math.atan(m)))
#		 except:
#			 self.ang1 = 90

#		 try:
#			 m = (x2[1] - x1[1])/(x2[0] - x1[0])
#			 self.ang2 = int(math.degrees(math.atan(-1/m)))
#		 except:
#			 self.ang2 = 90

#		 # cv2.putText(img, str(self.ang1), tuple(
#		 #	 p1), font, 2, (128, 255, 255), 3)
#		 # cv2.putText(img, str(self.ang2), tuple(
#		 #	 x1), font, 2, (255, 255, 128), 3)

#		 # print('div by zero error')
#	 def get_2d_points(self, img, camera_matrix, val):
#		 """Return the 3D points present as 2D for making annotation box"""
#		 point_3d = []
#		 self.dist_coeffs = np.zeros((4, 1))
#		 rear_size = val[0]
#		 rear_depth = val[1]
#		 point_3d.append((-rear_size, -rear_size, rear_depth))
#		 point_3d.append((-rear_size, rear_size, rear_depth))
#		 point_3d.append((rear_size, rear_size, rear_depth))
#		 point_3d.append((rear_size, -rear_size, rear_depth))
#		 point_3d.append((-rear_size, -rear_size, rear_depth))

#		 front_size = val[2]
#		 front_depth = val[3]
#		 point_3d.append((-front_size, -front_size, front_depth))
#		 point_3d.append((-front_size, front_size, front_depth))
#		 point_3d.append((front_size, front_size, front_depth))
#		 point_3d.append((front_size, -front_size, front_depth))
#		 point_3d.append((-front_size, -front_size, front_depth))
#		 point_3d = np.array(point_3d, dtype=float).reshape(-1, 3)

#		 # Map to 2d img points
#		 (point_2d, _) = cv2.projectPoints(point_3d,
#										   self.rotation_vector,
#										   self.translation_vector,
#										   self.camera_matrix,
#										   self.dist_coeffs) # returns 2d points as (x,y)
#		 point_2d = list(map(np.int32,point_2d.reshape(-1, 2))) # take in the x and y values of the 2d points and reshape them into a 2d array of points (x,y) for each point. the -1 means that the number of rows is inferred from the data provided,
#		 # and the 2 means that there are 2 columns (x,y) for each point. the map function takes in the np.int32() function and applies it to each element in the array so that the points are integers instead of floats (which is what the projectPoints function returns) because we need integers to draw the box around the face.
#		 # the output of this function is a list of tuples of the 2d points (x,y) for each point in the box around the face.
#		 return point_2d

#	 def draw_annotation_box(self, img, camera_matrix,
#							 rear_size=300, rear_depth=0, front_size=500, front_depth=400,
#							 color=(255, 255, 0), line_width=2):

#		 rear_size = 1
#		 rear_depth = 0
#		 front_size = img.shape[1]
#		 front_depth = front_size*2
#		 val = [rear_size, rear_depth, front_size, front_depth]
#		 point_2d = self.get_2d_points(img, self.camera_matrix, val)
#		 # # Draw all the lines
#		 # cv2.polylines(img, [point_2d], True, color, line_width, cv2.LINE_AA) # draw the box around the face
#		 # cv2.line(img, tuple(point_2d[1]), tuple(
#		 #	 point_2d[6]), color, line_width, cv2.LINE_AA) # draw line from left ear to right ear
#		 # cv2.line(img, tuple(point_2d[2]), tuple(
#		 #	 point_2d[7]), color, line_width, cv2.LINE_AA) # draw line from left eye to right eye
#		 # cv2.line(img, tuple(point_2d[3]), tuple(
#		 #	 point_2d[8]), color, line_width, cv2.LINE_AA) # draw line from left mouth to right mouth

#	 def head_pose_points(self, img, camera_matrix):
#		 """
#		 Get the points to estimate head pose sideways	

#		 Parameters
#		 ----------
#		 img : np.unit8
#			 Original Image.
#		 rotation_vector : Array of float64
#			 Rotation Vector obtained from cv2.solvePnP
#		 translation_vector : Array of float64
#			 Translation Vector obtained from cv2.solvePnP
#		 self.camera_matrix : Array of float64
#			 The camera matrix

#		 Returns
#		 -------
#		 (x, y) : tuple
#			 Coordinates of line to estimate head pose

#		 """
#		 rear_size = 1
#		 rear_depth = 0
#		 front_size = img.shape[1]
#		 front_depth = front_size*2
#		 val = [rear_size, rear_depth, front_size, front_depth]
#		 point_2d = self.get_2d_points(img, self.camera_matrix, val)
#		 y = (point_2d[5] + point_2d[8])//2
#		 x = point_2d[2]

#		 return (x, y)

#	 def head_rpy(self):
#		 '''Returns a list containing the head pose as a dictionary of roll, pitch, yaw angles, 
#			 and a string of the head pose direction'''
#		 head_rpy_dict = {'pitch': self.ang1, 'yaw': self.ang2}
#		 head_pos_str = ['Head:']
#		 if self.ang1 in range(-40, 40) and self.ang2 in range(-40, 40):
#			 head_pos_str.append('self.center')
#		 else:
#			 if self.ang1 >= 30:
#				 head_pos_str.append('down')
#				 # cv2.putText(img, 'Head down', (30, 30), font, 2, (255, 255, 128), 3)
#			 elif self.ang1 <= -30:
#				 head_pos_str.append('up')
#				 # cv2.putText(img, 'Head up', (30, 30), font, 2, (255, 255, 128), 3)

#			 if self.ang2 >= 40:
#				 head_pos_str.append('right')
#				 # cv2.putText(img, 'Head right', (90, 90), font, 2, (255, 255, 128), 3)
#			 elif self.ang2 <= -40:
#				 head_pos_str.append('left')
#				 # cv2.putText(img, 'Head left', (90, 90), font, 2, (255, 255, 128), 3)

#		 str_pose = ' '.join(head_pos_str)
#		 all_data = [str_pose, head_rpy_dict]
#		 return (all_data)

#	 def __str__(self) -> str:
#		 return str(self.head_rpy()[0][5:])

# //////////////////////////////////////////////////////////////////////////////

# class GazePosition(HeadPosition):
#	 print('in the GazePosition')
#	 def __init__(self, data):
#		 self.frame = data.frame
#		 print('3')
#		 super().__init__(data)
#		 print('head position super done')
#		 g = GazeTracking().feedback()
#		 self.left_center = g[0]
#		 self.right_center = g[1]
#		 self.left_corner = g[4]
#		 self.right_corner = g[5]
#		 self.top_corner = g[6]
#		 self.bottom_corner = g[7]
#		 self.frame = frame
#		 self.pupil_dir = "self.center"
#		 self.pupil_dir_y = " self.center"
#		 try:
#			 cv2.circle(self.frame, self.left_corner, 1, (0, 255, 0))
#			 cv2.circle(self.frame, self.right_corner, 1, (0, 255, 0))
#			 cv2.circle(self.frame, self.top_corner, 1, (0, 255, 0))
#			 cv2.circle(self.frame, self.bottom_corner, 1, (0, 255, 0))
#		 except:
#			 pass

#		 self.eye_width = tuple(
#			 i-j for i, j in zip(self.left_corner, self.right_corner))[0]
#		 self.eye_height = tuple(
#			 i-j for i, j in zip(self.top_corner, self.bottom_corner))[1]
#		 # cv2.circle(self.frame,self.right_center , self.eye_height, (0,255,0))

#	 def __str__(self):
#		 # isolate only the direction of the head (up,down,etc.)
#		 hp = super().__str__()
#		 detobj = self.dir_determin(hp)
  

#		 return detobj # returning the direction of the gaze

#	 def dir_determin(self, hp):
#		 if bool((math.fabs(self.right_center[0]-self.right_corner[0]) - self.eye_width/2) < -1):
#			 self.pupil_dir = "right"  # """Returns true if the user is looking to the right"""
#		 elif bool((math.fabs(self.right_center[0]-self.right_corner[0]) - self.eye_width/2) > 1):
#			 self.pupil_dir = 'left'  # """Returns true if the user is looking to the left"""


#		 if bool(math.fabs(math.fabs(self.right_center[1]-self.bottom_corner[1]) - self.eye_height/2) >= 1):
#			 self.pupil_dir_y = " up"  # """Returns true if the user is looking to the right"""
#		 else:
#			 self.pupil_dir_y = ' down'  # """Returns true if the user is looking to the left"""

#		 if self.pupil_dir in hp and self.pupil_dir_y in hp:
#			 return [self.pupil_dir+self.pupil_dir_y]

#		 else:
#			 return [self.pupil_dir+self.pupil_dir_y,' with head being ',hp]

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
			gazedir = GazeTracking().gazedirection(data)
			rospy.loginfo("returning the gaze direction to the client")
			return gazedir

		except BaseException as bex:
			rospy.loginfo(rospy.ServiceException(bex))

	

# //////////////////////////////////////////////////////////////////////////////

if __name__ == '__main__':
	try:
		GazeDetectorService()
		rospy.spin()
	except rospy.ROSInterruptException:
		pass
