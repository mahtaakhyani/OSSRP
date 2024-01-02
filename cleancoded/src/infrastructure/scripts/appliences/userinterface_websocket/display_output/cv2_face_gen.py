#!/usr/bin/env python3

import numpy as np
import cv2
import os
from sensor_msgs.msg import Image
from std_msgs.msg import String
import rospy


class CV2FaceGenerator:
    def __init__(self):
        rospy.init_node('convert_video_to_cv', anonymous=True)
        self.pub = rospy.Publisher('/cv2_3in1_face_publisher', Image, queue_size=10)
        rospy.loginfo('cv2_face_generator node started')
        self.sub = rospy.Subscriber('/exp_test', String, self.change_videos)
        rospy.loginfo('cv2_face_generator node subscribed to /exp_test')
        self.rate = rospy.Rate(10) # 10hz
        


    def convert_back(self, cv2_img):
        ros_image = Image()
        ros_image.header.stamp = rospy.Time.now()
        ros_image.height = cv2_img.shape[0]
        ros_image.width = cv2_img.shape[1]
        ros_image.encoding = "bgr8"
        ros_image.step = cv2_img.shape[1] * 3
        ros_image.data = np.array(cv2_img).tobytes()		
        return ros_image

    def run(self, img):
        while not rospy.is_shutdown():
            try:
                img_r = self.convert_back(img)
                self.pub.publish(img_r)
                rospy.loginfo('cv2_face_generator node published image to /cv2_3in1_face_publisher')
            except BaseException as e:
                print(e)
            self.rate.sleep()

    def change_videos(self, exp):
        rospy.loginfo('cv2_face_generator node received exp command')
        # get the path of the current directory
        path = os.path.dirname(os.path.realpath(__file__))

        video1 = f"{path}/happy_eyebrows.mp4"
        video2 = f"{path}/happy_eyes.mp4"
        video3 = f"{path}/happy_mouth.mp4"

        # get the videos
        if "eyes" in exp.data:
            video2 = f"{path}/{exp.data}_eyes.mp4"
        if "eyebrows" in exp.data:
            video1 = f"{path}/{exp.data}_eyebrows.mp4"
        if "mouth" in exp.data:
            video3 = f"{path}/{exp.data}_mouth.mp4"
        
        rospy.loginfo('cv2_face_generator node loaded videos')

        self.is_running = True
        self.show_video(video1, video2, video3)


    def show_video(self, vid1, vid2, vid3):
        rospy.sleep(0.5)
        rospy.loginfo(f'cv2_face_generator node started showing videos {vid1}, {vid2}, {vid3}')
        video1 = cv2.VideoCapture(vid1)
        video2 = cv2.VideoCapture(vid2)
        video3 = cv2.VideoCapture(vid3)
        video1.set(cv2.CAP_PROP_POS_FRAMES, 0)
        video2.set(cv2.CAP_PROP_POS_FRAMES, 0)
        video3.set(cv2.CAP_PROP_POS_FRAMES, 0)

        self.is_running = False
        # Get the dimensions of the videos
        width1 = int(video1.get(cv2.CAP_PROP_FRAME_WIDTH))
        height1 = int(video1.get(cv2.CAP_PROP_FRAME_HEIGHT))
        width2 = int(video2.get(cv2.CAP_PROP_FRAME_WIDTH))
        height2 = int(video2.get(cv2.CAP_PROP_FRAME_HEIGHT))
        width3 = int(video3.get(cv2.CAP_PROP_FRAME_WIDTH))
        height3 = int(video3.get(cv2.CAP_PROP_FRAME_HEIGHT))

        # Create a blank canvas to hold the combined videos
        canvas_width = width2 + height3
        canvas_height = height1 + height2 + height3
        canvas = np.zeros((canvas_height, canvas_width, 3), dtype=np.uint8)

        # Read frames from the videos and place them on the canvas
        # loop over the frames of the videos
        print(self.is_running)
        while True:
            if self.is_running:
                # release the videos
                video1.release()
                video2.release()
                video3.release()
                break
            # get the frames
            ret1, frame1 = video1.read()
            ret2, frame2 = video2.read()
            ret3, frame3 = video3.read()

            # check if the frames are empty
            if not ret1 or not ret2 or not ret3:
                video1.set(cv2.CAP_PROP_POS_FRAMES, 0)
                video2.set(cv2.CAP_PROP_POS_FRAMES, 0)
                video3.set(cv2.CAP_PROP_POS_FRAMES, 0)
                continue
            
            # Resize the frames to match the desired dimensions
            frame1 = cv2.resize(frame1, (width1, height1))
            frame2 = cv2.resize(frame2, (width2, height2))
            frame3 = cv2.resize(frame3, (width3, height3))

            # Place the frames on the canvas
            canvas[:height1, (canvas_width-width2)//2:(canvas_width-width2)//2+width1] = frame1
            canvas[height1:height1+height2, (canvas_width-width2)//2:(canvas_width-width2)//2+width2] = frame2
            canvas[height1+height2:, (canvas_width-width3)//2:(canvas_width-width3)//2+width3] = frame3

            img_r = self.convert_back(canvas)
            self.pub.publish(img_r)
            self.rate.sleep()
            # Display the canvas
            # cv2.imshow('Canvas', canvas)

            # wait for the user to press q
            # if cv2.waitKey(1) & 0xFF == ord('q'):
            #     break



if __name__ == '__main__':
    try:
        cv2_face_generator = CV2FaceGenerator()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass