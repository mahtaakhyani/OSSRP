#!/usr/bin/env python3.8
# Import Libraries
import cv2
import rospy
import mediapipe as mp
import numpy as np
from sensor_msgs.msg import Image, CameraInfo
from infrastructure.msg import List, Array3D, Landmarks
from std_msgs.msg import String
from geometry_msgs.msg import Point
from infrastructure.srv import Gaze
import time

# from .matrix import gridding as gd



class MeshDetector():
	rate = 20
	mp_holistic = mp.solutions.holistic
	image = np.zeros((640,480,3))
	imgmsg = Image()
	rospy.init_node("landmark_detection", anonymous=False)
	height = 480
	width = 640

	def __init__(self):
		self.t = time.time()
		self.dt = 0
		self.c =0
		rospy.Subscriber("/camera_info", CameraInfo, self.syncinfo, queue_size=10)
		# rospy.Subscriber('/image_cv2',List,self.catch)
		rospy.Subscriber("/image_raw", Image, self.count, callback_args=False, queue_size=1, buff_size=2**19) 
	
	
	def syncinfo(self, info):  # sync camera video stream info
		self.height = info.height
		self.width = info.width	
		# rospy.loginfo("image height: %s width: %s"%(self.height,self.width))
		try:
			return self.height, self.width
		except:
			pass	
	
	def count(self,s,d=False):
		self.dt = time.time() - self.t
		self.c +=1
		print('frame number ',self.c, self.dt)
		self.dt = 0
		self.t = time.time()
		# time.sleep(1.4)
		self.convert_frame(s,d)
		# self.reset_buffer()

	def convert_frame(self, data, cv2window=False): # convert the frame to a numpy array from ROS image
		self.t = time.time()

		# try:
		encoding = data.encoding
		dd = data
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
		self.dt = time.time() - self.t
		self.t = time.time()
		print('did preprocessing in ',self.dt)
		# rospy.loginfo("Pose: Got the image")

		self.arrrrr= np.array(self.modif_image, dtype=np.uint8).reshape((640,480,3))

		landmarks_array = self.analyze(self.arrrrr)
		# rospy.loginfo("Pose: Analyzed the image")
		

		face_points = []
		if landmarks_array[0]:
			for point in landmarks_array[0].landmark:
				point_sub_msg = Point()
				point_sub_msg.x = point.x
				point_sub_msg.y = point.y
				point_sub_msg.z = point.z
				face_points.append(point_sub_msg)


		lhand_points = []
		if landmarks_array[1]:
			for point in landmarks_array[1].landmark:
				point_sub_msg = Point()
				point_sub_msg.x = point.x
				point_sub_msg.y = point.y
				point_sub_msg.z = point.z
				lhand_points.append(point_sub_msg)


		rhand_points = []
		if landmarks_array[2]:
			for point in landmarks_array[2].landmark:
				point_sub_msg = Point()
				point_sub_msg.x = point.x
				point_sub_msg.y = point.y
				point_sub_msg.z = point.z
				rhand_points.append(point_sub_msg)

		pose_points = []
		if landmarks_array[3]:
			for point in landmarks_array[0].landmark:
				point_sub_msg = Point()
				point_sub_msg.x = point.x
				point_sub_msg.y = point.y
				point_sub_msg.z = point.z
				pose_points.append(point_sub_msg)

		landmarks_msg = Landmarks()
		landmarks_msg.face = face_points
		landmarks_msg.left_hand = lhand_points
		landmarks_msg.right_hand = rhand_points
		landmarks_msg.pose = pose_points

		
		landmark_pub = rospy.Publisher('/landmarks',Landmarks,queue_size=10)
		landmark_pub.publish(landmarks_msg)
		
		self.callservice(dd,landmarks_msg)

		# except Exception as e:
		# 	rospy.logerr("Error: %s"%e)
		
	def convert_back(self,cv2_img):
		ros_image = Image()
		ros_image.header.stamp = rospy.Time.now()
		ros_image.height = cv2_img.shape[0]
		ros_image.width = cv2_img.shape[1]
		ros_image.encoding = "bgr8"
		ros_image.step = cv2_img.shape[1] * 3
		ros_image.data = np.array(cv2_img).tobytes()		
		return ros_image


	def analyze(self,frame): #landmarks dtection -> face+pose+hands
		self.image = cv2.resize(frame, (self.height, self.width))
		# # Convert the image to 8-bit unsigned integer data type
		# image_8u = cv2.convertScaleAbs(self.image)

		# Convert the image to a NumPy array with three color channels
		image_array = cv2.cvtColor(self.image, cv2.COLOR_BGR2RGB)
		with self.mp_holistic.Holistic(
			# max_num_faces=1,  # number of faces to track in each frame
			refine_face_landmarks=True,  # includes iris landmarks in the face mesh model
			min_detection_confidence=0.5,
			min_tracking_confidence=0.5) as face_mesh:
			results = face_mesh.process(image_array)
		self.dt = time.time() - self.t
		self.t = time.time()
		print('mesh results took ',self.dt)
		# gathering all landmarks for global use
		self.face_landmarks = (results.face_landmarks , self.mp_holistic.FACEMESH_TESSELATION )
		self.right_hand_landmarks = (results.right_hand_landmarks, self.mp_holistic.HAND_CONNECTIONS)
		self.left_hand_landmarks = (results.left_hand_landmarks, self.mp_holistic.HAND_CONNECTIONS)
		self.pose_landmarks = (results.pose_landmarks, self.mp_holistic.POSE_CONNECTIONS)


		# Draw landmarks and connections between them
		self.draw(self.face_landmarks, thickness=1, color=(125,125,125))
		self.draw(self.right_hand_landmarks, radius=5)
		self.draw(self.left_hand_landmarks, radius=5)
		self.draw(self.pose_landmarks, thickness=1, radius=2, color=(0,255,255))

		te = Array3D()
		tem = List()
		temp = List()
		msg = List()
		pub = rospy.Publisher('/image_raw/landmarked',Image,queue_size=10)
		pubcv2 = rospy.Publisher('/image_cv2/landmarked', List, queue_size=10)
		
		arrrrr= np.array(self.image, dtype=np.uint8).reshape((640,480,3)) # convert the list of lists to a numpy array
		self.msg = self.convert_back(arrrrr)
		pub.publish(self.msg)

		# llll=self.image.tolist()
		# tem.data = [Array3D(j) for i in llll for j in i] # convert the image to a list of 3D arrays 
		# # rospy.loginfo("Pose: Converted the image to a list of 3D arrays")
		# self.dt = time.time() - self.t
		# self.t = time.time()
		# print('converting messages ',self.dt)
		# arr = [list(item.data) for item in tem.data] # convert the list of 3D arrays to a list of lists since the 3D array is not supported in ROS
		# pubcv2.publish(tem)
		
		# Returning the processed image back to the main module
		return [results.face_landmarks,results.left_hand_landmarks,results.right_hand_landmarks,results.pose_landmarks]

	def pointing(self, image):

		return image
		
	def callservice(self,frame,landmarks_msg):
		pub = rospy.Publisher('gaze_position/gaze_dir', String, queue_size=10)
		rospy.loginfo("waiting for gaze pose service to respond...")
		rospy.wait_for_service('gaze_pose')
		try:
			gazsrv = rospy.ServiceProxy('gaze_pose', Gaze)
			rospy.loginfo("gaze pose service responded.")
			transcript = gazsrv(frame, landmarks_msg)
			rospy.loginfo("gaze_pose server successfully responded with %s",transcript.gazedirection)
			rospy.loginfo("Publishing gaze position to /gaze_position/gaze_dir topic")
			pub.publish(transcript.gazedirection)
	        
		except rospy.ServiceException as e:
			rospy.logerr("Service call failed: %s"%e)

	def draw(self, landmark, thickness=2 ,radius=0, color=(255,0,255)):

				# Initializing the drawing utils for drawing the facial landmarks on image
			mp_drawing = mp.solutions.drawing_utils
			# Drawing the Facial Landmarks
			mp_drawing.draw_landmarks(
			self.image,
			landmark[0],
			landmark[1],
			mp_drawing.DrawingSpec(
				color=color[::-1],
				thickness=0,
				circle_radius=radius
			),
			mp_drawing.DrawingSpec(
				color=color,
				thickness=thickness,
				circle_radius=radius
			)
			)
			return self.image

	


if __name__ == "__main__":
		md = MeshDetector()
		rospy.spin()

