import cv2
import mediapipe as mp
import numpy as np
import time
import pandas as pd
from datetime import datetime
from deepface import DeepFace
import logging
logging.basicConfig(level=logging.CRITICAL)

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

		e_lx, e_ly = 0, 0
		e_rx, e_ry = 0, 0
		eyebrow_ly2, eyebrow_lx2, eyebrow_ry2, eyebrow_rx2 = 0, 0, 0, 0
		pupil_lx , pupil_ly, pupil_rx, pupil_ry = 0, 0, 0, 0

		eyebrow_offset_height = 150
		eyebrow_offset_width = 250

		pupil_scale = 2
		eyebrow_scale = 3

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
				time.sleep(0.1)
				try:
					ret, frame = cap.read()
					if not ret:
						print("Failed to capture frame")
						df = pd.DataFrame(data, columns=data_headers)
						# df.to_excel(self.data_file, index=False)
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
							right_left_corner = mesh_points[133]

							# an empty numpy array the same size as the frame
							canvas = np.ones((frame.shape[0], frame.shape[1], 3), dtype=np.uint8) * 255
							
							pupil_image = cv2.imread(f'{self.dir}/pupil.png')
							# resize the pupil.png to half the size
							pupil_image = cv2.resize(pupil_image, (int(pupil_image.shape[1] / pupil_scale), int(pupil_image.shape[0] / pupil_scale)))
							pupil_width = pupil_image.shape[1]
							pupil_height = pupil_image.shape[0]

							# the same for eyebrow_left.png
							eyebrow_image = cv2.imread(f'{self.dir}/eyebrow_left.png')
							eyebrow_image = cv2.resize(eyebrow_image, (int(eyebrow_image.shape[1] / eyebrow_scale), int(eyebrow_image.shape[0] / eyebrow_scale)))
							eyebrow_image_right = cv2.flip(eyebrow_image, 1)
							
							eyebrow_width = eyebrow_image.shape[1]
							eyebrow_height = eyebrow_image.shape[0]


							
							if frame_number==1:
								# left pupil
								lx1 = int(img_w/2)-pupil_width
								ly1 = int(2*img_h/3)-pupil_height
								lx2 = lx1 + pupil_width
								ly2 = ly1 + pupil_height
								canvas[ly1: ly2, lx1: lx2] = pupil_image
								
								# right pupil
								rx1 = int(img_w/2)+pupil_width
								ry1 = int(2*img_h/3)-pupil_height
								rx2 = rx1 + pupil_width
								ry2 = ry1 + pupil_height
								canvas[ry1: ry2, rx1: rx2] = pupil_image

								# pupils' centers
								pupil_lx = int(l_cx)
								pupil_ly = int(l_cy)
								pupil_rx = int(r_cx)
								pupil_ry = int(r_cy)

								# left eyebrow
								# the eyebrow is above the eye, so the y coordinate is less than the eye's y coordinate
								eyebrow_lx1 = int(img_w/2)-eyebrow_width
								eyebrow_ly1 = int(2*img_h/3)-pupil_height-eyebrow_height
								eyebrow_lx2 = eyebrow_lx1 + eyebrow_width
								eyebrow_ly2 = eyebrow_ly1 + eyebrow_height
								canvas[eyebrow_ly1: eyebrow_ly2, eyebrow_lx1: eyebrow_lx2] = eyebrow_image

								# right eyebrow
								eyebrow_rx1 = int(img_w/2)+eyebrow_width
								eyebrow_ry1 = int(2*img_h/3)-pupil_height-eyebrow_height
								eyebrow_rx2 = eyebrow_rx1 + eyebrow_width
								eyebrow_ry2 = eyebrow_ry1 + eyebrow_height
								canvas[eyebrow_ry1: eyebrow_ry2, eyebrow_rx1: eyebrow_rx2] = eyebrow_image_right

								# eyebrows' centers (mesh_points[105] is the right eyebrow's center and mesh_points[334] is the left one)
								e_lx = int(mesh_points[334][0])
								e_ly = int(mesh_points[334][1])
								eyebrow_ly2 = img_h/3 - pupil_height + eyebrow_height
								e_rx = int(mesh_points[105][0])
								e_ry = int(mesh_points[105][1])

								# eyebrow's corners (mesh_points[46] is the left eyebrow's corner and mesh_points[276] is the right one)
								ec_lx = int(mesh_points[46][0])
								ec_ly = int(mesh_points[46][1])
								ec_rx = int(mesh_points[276][0])
								ec_ry = int(mesh_points[276][1])


							else:
								
								try:
									# Place the center of the pupil.png on the canvas new_image on the lx_cx, ly_cy, withouth resizing the original pupil.png
									# calculate where the center of the pupil.png should be
									# pupils
									threshhold = 60
									pupil_ldx = int(l_cx - pupil_lx)*2
									if abs(pupil_ldx) < pupil_width/threshhold:
										pupil_ldx = 0
									pupil_ldy = int(l_cy - pupil_ly)*2
									if abs(pupil_ldy) < pupil_height/threshhold:
										pupil_ldy = 0
									pupil_rdx = int(r_cx - pupil_rx)*2
									if abs(pupil_rdx) < pupil_width/threshhold:
										pupil_rdx = 0
									pupil_rdy = int(r_cy - pupil_ry)*2
									if abs(pupil_rdy) < pupil_height/threshhold:
										pupil_rdy = 0
										
									# lx2 = lx1 + pupil_ldx
									# ly2 = int(2*img_h/3) + pupil_ldy
									# rx2 = rx1 + pupil_rdx
									# ry2 = int(2*img_h/3) + pupil_rdy
									
									lx2 = int(l_cx) - pupil_width
									ly2 = int(l_cy) + int(pupil_height/2)
									rx2 = int(r_cx)	+ pupil_width
									ry2 = int(r_cy) + int(pupil_height/2)
									if lx2>frame.shape[1] or ly2>frame.shape[0] or rx2>frame.shape[1] or ry2>frame.shape[0]:
										lx2 = lx1
										ly2 = ly1
										rx2 = rx1
										ry2 = ry1

									canvas[ly2-pupil_height:ly2, lx2-pupil_width:lx2] = pupil_image
									canvas[ry2-pupil_height:ry2, rx2-pupil_width:rx2] = pupil_image

									# new pupil centers
									l_cx = lx2/2
									l_cy = ly2/2
									r_cx = rx2/2
									r_cy = ry2/2

									time.sleep(0.1)
									# eyebrows
									eyebrow_ldx = int(mesh_points[334][0] - e_lx)
									eyebrow_ldy = int(mesh_points[334][1] - e_ly)*3
									eyebrow_rdx = int(mesh_points[105][0] - e_rx)*2
									eyebrow_rdy = int(mesh_points[105][1] - e_ry)*2


									eyebrow_lx2 = e_lx + eyebrow_ldx
									eyebrow_ly2 = e_ly + eyebrow_ldy
									eyebrow_rx2 = e_rx - eyebrow_rdx
									eyebrow_ry2 = e_ry - eyebrow_rdy

									print(eyebrow_ly2, eyebrow_ldy)

									if eyebrow_lx2>frame.shape[1] or eyebrow_ly2>frame.shape[0] or eyebrow_rx2>frame.shape[1] or eyebrow_ry2>frame.shape[0]:
										eyebrow_lx2 = e_lx
										eyebrow_ly2 = int(2*img_h/3)-pupil_height + eyebrow_height
										eyebrow_rx2 = e_rx
										eyebrow_ry2 = e_ry

									
									
									canvas[eyebrow_ly2-eyebrow_height: eyebrow_ly2, eyebrow_lx2-eyebrow_width: eyebrow_lx2] = eyebrow_image
									# canvas[eyebrow_ry2-eyebrow_height: eyebrow_ry2, eyebrow_rx2-eyebrow_width: eyebrow_rx2] = eyebrow_image_right
									
									e_lx = mesh_points[334][0]
									e_ly = mesh_points[334][1]
									e_rx = mesh_points[105][0]
									e_ry = mesh_points[105][1]
									

								except BaseException as e:
									print(e)
									# canvas = np.ones((frame.shape[0], frame.shape[1], 3), dtype=np.uint8) * 255
									# lx2 = lx1 + pupil_width
									# ly2 = int(2*img_h/3) + pupil_height
									# rx2 = rx1 + pupil_width
									# ry2 = int(2*img_h/3) + pupil_height
									# canvas[ly1: ly2, lx1: lx2] = pupil_image
									# canvas[ry1: ry2, rx1: rx2] = pupil_image

									# eyebrow_lx2 = eyebrow_lx1 + eyebrow_width
									# eyebrow_ly2 = int(2*img_h/3)-pupil_height + eyebrow_height
									# eyebrow_rx2 = eyebrow_rx1 - eyebrow_width
									# eyebrow_ry2 = int(2*img_h/3)-pupil_height + eyebrow_height
									# canvas[eyebrow_ly1-eyebrow_height: eyebrow_ly2, eyebrow_lx1: eyebrow_lx2] = eyebrow_image
									# canvas[eyebrow_ry1-eyebrow_height: eyebrow_ry2, eyebrow_rx1: eyebrow_rx2] = eyebrow_image_right





							# Display the result
							cv2.imshow("Canvas with Pupil", canvas)
							

							
							
							cv2.circle(frame, right, int(r_radius), (0, 255, 0))
							cv2.circle(frame, mesh_points[390], 6, (0, 255, 0))
							cv2.circle(frame, right_left_corner, 6, (0, 255, 0))
							cv2.circle(frame, center_left, int(lc_radius), (255, 0, 255))
							cv2.circle(frame, center_right, int(rc_radius), (255, 0, 255))
						

						if camera:
							out.write(frame)
							time.sleep(frame_delay)

						# cv2.imshow('img', frame)
						key = cv2.waitKey(1)
						if key == ord('q'):
							break
					except BaseException as e:
						current_time = datetime.now().strftime('%H:%M:%S')
						data.append([current_time,real_frame_number, "No face detected", ''])
						# print(e)

				except KeyboardInterrupt:
					df = pd.DataFrame(data, columns=data_headers)
					# df.to_excel(self.data_file, index=False)
					break

		cap.release()
		cv2.destroyAllWindows()
		df = pd.DataFrame(data, columns=data_headers)
		# df.to_excel(self.data_file, index=False)

if __name__ == '__main__':
	dir = os.path.dirname(os.path.abspath(__file__))
	gaze_detector = GazeDetector(video_path=f'{dir}/vid.mp4', iris_diameter_mm=11.8, focal_length_mm=24)

	gaze_detector.analyze(camera=True)
