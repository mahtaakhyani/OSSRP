#!/usr/bin/env python3.8
import cv2
from statistics import mode
import random
from deepface import DeepFace
import rospy
from infrastructure.msg import FaceEmotions, List, EmoProbArr, Array3D
from sensor_msgs.msg import CameraInfo,Image
import numpy as np
import mediapipe as mp
import time
import matplotlib.pyplot as plt



# Face Emotion Analysis publisher node
# Taking the camera input in, analyze it, 
# then publish it on the related topic through a 2D array message 
# containing the list of names and the probability of the given emotion being the answer
# (The higher the number, the stronger assurance)

class FaceDetection():  
	mp_face_mesh = mp.solutions.face_mesh  # initialize the face mesh model
	mp_holistic = mp.solutions.holistic
	mp_drawing = mp.solutions.drawing_utils
	frame_msgs = None
	face_cascade_name = None
	face_cascade = None
	img = None
	face_width = None
	height = 640
	width = 480	
	global_dominant_emotion = []
	submsg = EmoProbArr()
	msg = FaceEmotions()
	pub = rospy.Publisher('/face_emotions',FaceEmotions, queue_size=10)
	rospy.init_node('FaceEmotionAnalysis',anonymous=False)

	def __init__(self):
		self.dt = 0
		# self.newb = 2
		# self.p = 1
		# self.mindt = 0
		# self.lastemo = ''
		# self.ts = []
		# self.bs = []
		# self.mean = []
		# self.num = 0
		self.c = 0
		self.t = time.time()
		rospy.Subscriber("/camera_info", CameraInfo, self.syncinfo, queue_size=10)
		self.sub = rospy.Subscriber("/image_raw", Image, self.count, callback_args=False, queue_size=10, buff_size=2*13)
	
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
		self.convert_frame(s,d)
		# self.reset_buffer()
	
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

				self.arrrrr= np.array(self.modif_image, dtype=np.uint8).reshape((640,480,3))
				self.feedback(self.arrrrr)
		

			except Exception as e:
				rospy.logerr("Error: %s"%e)
				return e



	def prequisites(self,image):
		# image = cv2.resize(image, (800, 600))
		# image.flags.writeable = False
		# gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
		try:
			faces = DeepFace.detectFace(image, detector_backend='opencv')
			# for x,y,w,h in faces:
			# 			self.img=cv2.rectangle(self.frame_msgs,(x,y),(x+w,y+h),
			# 			(random.randint(0,255),random.randint(0,255),random.randint(0,255)),1)  #making a recentangle to show up and detect the face and setting it position and colour
			# 			self.face_width = h
			print("Face Detected")
			return True
		except:
			print("No face detected")
			return False

		with self.mp_holistic.Holistic(
		# max_num_faces=1,  # number of faces to track in each frame
		refine_face_landmarks=True,  # includes iris landmarks in the face mesh model
		min_detection_confidence=0.5,
		min_tracking_confidence=0.5) as face_mesh:
		
			
			image = cv2.resize(image, (800, 600))
			# To improve performance, optionally mark the image as not writeable to
			# pass by reference.
			image.flags.writeable = False
			image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)  # frame to RGB for the face-mesh model
			results = face_mesh.process(image)
			image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)  # frame back to BGR for OpenCV
			if not results.face_landmarks:
				print("No face detected")
			else: 

				print("Face Detected")
				self.frame_msgs = image
				self.face_cascade_name = cv2.data.haarcascades + 'haarcascade_frontalface_alt.xml'  #getting a haarcascade xml file
				self.face_cascade = cv2.CascadeClassifier()  #processing it for our project
				if not self.face_cascade.load(cv2.samples.findFile(self.face_cascade_name)):  #adding a fallback event
					print("Error loading xml file")
				gray = cv2.cvtColor(self.frame_msgs, cv2.COLOR_BGR2GRAY)
				face=self.face_cascade.detectMultiScale(gray,scaleFactor=1.1,minNeighbors=5)
				
				for x,y,w,h in face:
						self.img=cv2.rectangle(self.frame_msgs,(x,y),(x+w,y+h),
						(random.randint(0,255),random.randint(0,255),random.randint(0,255)),1)  #making a recentangle to show up and detect the face and setting it position and colour
						self.face_width = h
				
				self.dt = time.time() - self.t
				self.t = time.time()
				print('time until face has been detected since the frame was caught: ',self.dt)
				return True
			

	def analysis(self,image):
			condition = self.prequisites(image)
			while not condition:
				condition = self.prequisites(image)
				return "No face detected."
			else:
				analyze = DeepFace.analyze(image,actions=['emotion'], enforce_detection=False)  #same thing is happing here as the previous example, we are using the analyze class from deepface and using ‘frame’ as input
				result = [analyze]
				
				self.dt = time.time() - self.t
				self.t = time.time()
				print('from catch until analyzed: ',self.dt)
				return result


	def feedback(self,image):  
		
		image = cv2.resize(image,(self.height,self.width))
		feedback = self.analysis(image)
		if feedback != "No face detected":
			emotion = feedback[0]['emotion']
			dominant = feedback[0]['dominant_emotion']
			self.dt = time.time() - self.t

			print('analyzed untill last process: ',self.dt)

			# self.dt = time.time() - self.t
			# self.t = time.time()
			# mean = self.dt
			# self.num+=1
			# if self.num != 2:
			# 	self.mean.append(self.dt)
			# else:

			# 	mean = sum(self.mean)/len(self.mean)
			# 	print(self.num,'----------------------------',mean)
			# 	st = str(self.newb)+'**'+str(self.p)
			# 	self.bs.append(st)
			# 	self.ts.append(mean)
			# 	self.mindt = min(self.ts)
			# 	if  mean > self.mindt:
			# 		self.reset_buffer()
			# 	self.mean = []
			# 	self.num = 0

			if len(self.global_dominant_emotion) <= 3:
				self.global_dominant_emotion.append(str(dominant))
			else:
				self.global_dominant_emotion = []
				self.global_dominant_emotion.append(str(dominant))

			overall_emotion = mode(self.global_dominant_emotion)
			# Sorting emotions by higher propablity values gotten
			sorted_values = sorted(emotion.items(), key=lambda item: item[1])[::-1]
			prettied_dict = [str(i)+' : '+str(round(j,6)) for i,j in sorted_values]+['Dominant Emotion : '+str(overall_emotion)]
			temp = []

			for item in sorted_values:
				self.submsg = EmoProbArr()
				self.submsg.emotion, self.submsg.probability = item[0], float(round(item[1],6))
				temp.append(self.submsg)
			self.msg.data = temp
			self.pub.publish(self.msg)
			self.msg.data = []
			# rospy.Rate(5).sleep()
			# Converting to string to display on the camera frame output
			prettied_str = '\n'.join(prettied_dict)
			rospy.loginfo(prettied_str)
			self.t = time.time()
			return prettied_str
		else:
			self.t = time.time()
			self.msg.data = []
			self.pub.publish(self.msg)
			return "No face detected"
		
	# def reset_buffer(self):
	# 	self.sub.unregister()
	# 	# print('unregistered from topic. min dt: ',self.mindt)
	# 	self.sub = rospy.Subscriber("/image_raw", Image, self.count, callback_args=False, queue_size=1, buff_size=self.newb**self.p)
	# 	print('subscribed to the topic with buffer size: ',self.newb,'**',self.p)
	# 	self.p +=1

	# 	if self.p > 13:
	# 		self.newb+=1
	# 		self.p = 1

	# def __str__(self):
	# 	return [self.ts,self.bs]
			


if __name__ == '__main__':
	d = FaceDetection().__str__()
	rospy.spin()
	# a=d[1]
	# b=d[0]
	# print(a,b)
	# plt.plot(a,b)
	# plt.show()
