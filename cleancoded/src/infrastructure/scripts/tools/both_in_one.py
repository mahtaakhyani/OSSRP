import cv2
import mediapipe as mp
import numpy as np
import time
import pandas as pd
from datetime import datetime
from deepface import DeepFace

import os, sys

class GazeDetector:
	def __init__(self, video_path, iris_diameter_mm, focal_length_mm):
		self.video_path = video_path
		self.iris_diameter_mm = iris_diameter_mm
		self.focal_length_mm = focal_length_mm
		self.mp_face_mesh = mp.solutions.face_mesh
		self.LEFT_EYE = [362, 382, 381, 380, 374, 373, 390, 249, 263, 466, 388, 387, 386, 385, 384, 398]
		self.RIGHT_EYE = [33, 7, 163, 144, 145, 153, 154, 155, 133, 173, 157, 158, 159, 160, 161, 246]
		self.LEFT_IRIS = [474, 475, 476, 477]
		self.RIGHT_IRIS = [469, 470, 471, 472]
		self.dir = os.path.dirname(os.path.abspath(__file__))
		self.data_file = f'{self.dir}/extracted_data_all.xlsx'

	def analyze(self, camera=True):
		if camera:
			cap = cv2.VideoCapture(0)
		else:
			cap = cv2.VideoCapture(self.video_path)
			cap.set(cv2.CAP_PROP_POS_FRAMES, 0)

		if not cap.isOpened():
			print("Failed to open camera/ video feed")
			return

		distance = 0
		frame_number = 0
		real_frame_number = 0
		fourcc = cv2.VideoWriter_fourcc(*'mp4v')
		width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
		height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
		frame_rate = cap.get(cv2.CAP_PROP_FPS)
		frame_delay = 1 / frame_rate

		if camera:
			out = cv2.VideoWriter(f'{self.dir}/recorded_video_gaze.mp4', fourcc, frame_rate, (width, height))

		data_headers = ['Time', 'dT', 'Gaze', 'Head', 'Emotion']
		data = []

		with self.mp_face_mesh.FaceMesh(
				max_num_faces=1,
				refine_landmarks=True,
				min_detection_confidence=0.5,
				min_tracking_confidence=0.5
		) as face_mesh:
			while True:
				try:
					ret, frame = cap.read()
					if not ret:
						print("Failed to capture frame")
						df = pd.DataFrame(data, columns=data_headers)
						df.to_excel(self.data_file, index=False)
						break

					frame_number += 1
					real_frame_number += 1 * (1 / frame_rate)
					frame = cv2.flip(frame, 1)
					rgb_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
					img_h, img_w = frame.shape[:2]
					try:
						faces = DeepFace.extract_faces(frame, detector_backend='opencv')
						results = face_mesh.process(rgb_frame)

						if results.multi_face_landmarks:
							mesh_points = np.array(
								[np.multiply([p.x, p.y], [img_w, img_h]).astype(int) for p in
								results.multi_face_landmarks[0].landmark])

							for point in mesh_points:
								cv2.circle(frame, (int(point[0]), int(point[1])), 2, (0, 255, 0))

							(l_cx, l_cy), lc_radius = cv2.minEnclosingCircle(mesh_points[self.LEFT_IRIS])
							(r_cx, r_cy), rc_radius = cv2.minEnclosingCircle(mesh_points[self.RIGHT_IRIS])
							(r_x, r_y), r_radius = cv2.minEnclosingCircle(mesh_points[self.LEFT_EYE])
							center_left = np.array([l_cx, l_cy], dtype=np.int32)
							center_right = np.array([r_cx, r_cy], dtype=np.int32)

							right = np.array([r_x, r_y], dtype=np.int32)
							right_right_corner = mesh_points[33]
							right_left_corner = mesh_points[133]


							left_left_corner = mesh_points[263]
							left_right_corner = mesh_points[362]

							plc = np.linalg.norm(right_left_corner - center_right)
							plr = np.linalg.norm(left_right_corner - center_left)

							if plr+r_radius/4 < plc:
								gaze_position = 'left'
							elif plr-r_radius/4 > plc:
								gaze_position = 'right'
							else:
								gaze_position = 'center horizontally'

								
							rcc = mesh_points[353]
							lcc = mesh_points[124]

							x,y = mesh_points[386]
							equation = (rcc[1] - lcc[1]) * x - (rcc[0] - lcc[0]) * y + rcc[0] * lcc[1] - rcc[1] * lcc[0]
							print(r_radius)
							cv2.line(frame, tuple(left_left_corner), tuple(right_right_corner), (0, 255, 0), 2)

							if equation-r_radius*10>0:
								gaze_position += ' up'
							elif equation+r_radius*30<0:
								gaze_position += ' down'
							else:
								gaze_position += ' center vertically'
							
							cv2.circle(frame, right, int(r_radius), (0, 255, 0))
							cv2.circle(frame, mesh_points[390], 6, (0, 255, 0))
							cv2.circle(frame, right_left_corner, 6, (0, 255, 0))
							cv2.circle(frame, center_left, int(lc_radius), (255, 0, 255))
							cv2.circle(frame, center_right, int(rc_radius), (255, 0, 255))

							cv2.putText(frame, gaze_position, (50, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)

							iris_diameter_pixels = 2 * r_radius
							estimated_distance_mm = (self.iris_diameter_mm * self.focal_length_mm) / iris_diameter_pixels
							moved_distance = 0

							if frame_number == 1:
								distance = estimated_distance_mm
							else:
								moved_distance = int(estimated_distance_mm - distance) * 10

							if moved_distance > 0:
								dtext = "farther from"
							else:
								dtext = "closer to"

							cv2.putText(frame, f"Moved {moved_distance} cm {dtext} the starting point", (10, 100),
										cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)


							chick_center = mesh_points[36]
							chick_left = mesh_points[137]
							chick_right = mesh_points[366]
							chick_right_center = mesh_points[266]
							p1 = mesh_points[10]
							p2 = mesh_points[4]
							p3 = mesh_points[152]
							p12 = np.linalg.norm(p1 - p2)
							p23 = np.linalg.norm(p2 - p3)

							cur_chick_len = np.linalg.norm(chick_left - chick_center)
							cur_chick_right_len = np.linalg.norm(chick_right - chick_right_center)
							if p23-20 > p12:
								orientation = "up"
							elif p12-40 > p23:
								orientation = "down"
							else:
								orientation = "center vertically"

							if cur_chick_len < cur_chick_right_len -5:
								orientation += ", left"
							elif cur_chick_len > cur_chick_right_len +5:
								orientation += ", right"
							else:
								orientation += ", center horizontally"

							cv2.putText(frame, "Head Orientation: {}".format(orientation),
										(10, 200), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
							analyze = DeepFace.analyze(frame, actions=['emotion'], enforce_detection=False)
							emotions = analyze[0]['emotion']
							dominant = analyze[0]['dominant_emotion']
							print(f'\ndt: {frame_number}\nDominant: {dominant} \nEmotions:\n{emotions}')
							current_time = datetime.now().strftime('%H:%M:%S')
							data.append([current_time, real_frame_number, gaze_position, orientation, dominant])

						if camera:
							out.write(frame)
							time.sleep(frame_delay)

						cv2.imshow('img', frame)
						key = cv2.waitKey(1)
						if key == ord('q'):
							break
					except BaseException as e:
						current_time = datetime.now().strftime('%H:%M:%S')
						data.append([current_time,real_frame_number, "No face detected", ''])
						print(e)

				except KeyboardInterrupt:
					df = pd.DataFrame(data, columns=data_headers)
					df.to_excel(self.data_file, index=False)
					break

		cap.release()
		cv2.destroyAllWindows()
		df = pd.DataFrame(data, columns=data_headers)
		df.to_excel(self.data_file, index=False)

if __name__ == '__main__':
	dir = os.path.dirname(os.path.abspath(__file__))
	gaze_detector = GazeDetector(video_path=f'{dir}/vid.mp4', iris_diameter_mm=11.8, focal_length_mm=24)

	gaze_detector.analyze(camera=True)
