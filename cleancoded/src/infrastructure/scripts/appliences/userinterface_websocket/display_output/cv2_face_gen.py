import numpy as np
import cv2
import os
from convert_video_to_cv import ConvertVideoToCv

def show_video():
    convert = ConvertVideoToCv()
    # get the path of the current directory
    path = os.path.dirname(os.path.realpath(__file__))

    # get the videos
    video1 = cv2.VideoCapture(f"{path}/1.mp4")
    video2 = cv2.VideoCapture(f"{path}/2.mp4")
    video3 = cv2.VideoCapture(f"{path}/3.mp4")

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
    while True:
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

        # Display the canvas
        cv2.imshow('Canvas', canvas)
        convert.run(canvas)

        # wait for the user to press q
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
        
    # release the videos
    video1.release()
    video2.release()
    video3.release()

    # close all windows
    cv2.destroyAllWindows()



show_video()

