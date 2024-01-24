import cv2
import time
from datetime import datetime
from deepface import DeepFace
import random
import os, sys
import pandas as pd
import heapq

class LogAnalyzer:
	def __init__(self, video_path, data_file):
		self.video_path = video_path
		self.data_file = data_file
		self.data_headers = ['Time','dT' ,'Emotion', 'Probability %']
		self.data = []

	def analyze(self, camera=True):
		
		frame_number = 0
		if camera:
			cap = cv2.VideoCapture(0)
		else:
			cap = cv2.VideoCapture(self.video_path)
			cap.set(cv2.CAP_PROP_POS_FRAMES, 0)

		if not cap.isOpened():
			print("Failed to open camera/ video feed")
			return

		# Define the codec and create a VideoWriter object
		fourcc = cv2.VideoWriter_fourcc(*'mp4v')
		width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
		height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
		frame_rate = cap.get(cv2.CAP_PROP_FPS)
		frame_delay = 1 / frame_rate

		if camera:
			out = cv2.VideoWriter(f'{dir}/recorded_video.mp4', fourcc, frame_rate, (width, height))

		while True:
				ret, frame = cap.read()
				if not ret:
					print("Failed to capture frame")
					df = pd.DataFrame(self.data, columns=self.data_headers)
					# Save the DataFrame to an Excel file
					df.to_excel(self.data_file, index=False)
					break

				# Get the default camera resolution
				cv2.imshow('Camera Input', frame)
				frame_number +=1*(1/frame_rate)
				if camera:
					# Write the frame to the file
					out.write(frame)
					time.sleep(frame_delay)
				try:
					try:
						faces = DeepFace.extract_faces(frame, detector_backend='opencv')         
						analyze = DeepFace.analyze(frame, actions=['emotion'], enforce_detection=False)
						emotions = analyze[0]['emotion']
						dominant = analyze[0]['dominant_emotion']
						print(f'\ndt: {frame_number}\nDominant: {dominant} \nEmotions:\n{emotions}')
						# Get the two maximum values from the dictionary
						max_values = heapq.nlargest(2, emotions.values())
						# Iterate over the dictionary to find the keys corresponding to the maximum values
						max_keys = [key for key, value in emotions.items() if value in max_values]
						for key, value in zip(max_keys, max_values):
							current_time = datetime.now().strftime('%H:%M:%S')
							self.data.append([current_time,frame_number, key, value])
						self.data.append(['----','----','----','----'])

						# Display the resulting frame
						print("Face Detected")
					except KeyboardInterrupt:
						# Create a DataFrame from the list of strings
						df = pd.DataFrame(self.data, columns=self.data_headers)
						# Save the DataFrame to an Excel file
						df.to_excel(self.data_file, index=False)
						break
				except:
					current_time = datetime.now().strftime('%H:%M:%S')
					self.data.append([current_time,frame_number, "No face detected", ''])
					self.data.append(['----','----','----','----'])
					print("No face detected")
					
				
					if cv2.waitKey(1) & 0xFF == ord('s'):
						break
			
				
				if camera:
					# Press 'q' to stop capturing
					if cv2.waitKey(1) & 0xFF == ord('q'):
						break
				
		# Release the VideoCapture and VideoWriter objects
		cap.release()
		# Destroy all windows
		cv2.destroyAllWindows()
		print(self.data)
		# Create a DataFrame from the list of strings
		df = pd.DataFrame(self.data, columns=self.data_headers)
		# Save the DataFrame to an Excel file
		df.to_excel(self.data_file, index=False)

if __name__ == "__main__":
	dir = os.path.dirname(os.path.abspath(__file__))
	analyzer = LogAnalyzer(video_path=f'{dir}/vid.mp4', data_file=f'{dir}/extracted_data.xlsx')
	analyzer.analyze(camera=False)
