import cv2
import time
from deepface import DeepFace

def save_camera_input_to_file(file_path):
    cap = cv2.VideoCapture(0)
    if not cap.isOpened():
        print("Failed to open camera")
        return
    
    # Get the default camera resolution
    width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    
    # Define the codec and create a VideoWriter object
    fourcc = cv2.VideoWriter_fourcc(*'mp4v')
    out = cv2.VideoWriter(file_path, fourcc, 20.0, (width, height))
    t = time.time()
    dt = 0
    while dt<5:

        dt = time.time() - t
        print(dt)
        ret, frame = cap.read()
        if not ret:
            print("Failed to capture frame")
            break
        
        # Write the frame to the file
        out.write(frame)
        try:
            faces = DeepFace.detectFace(frame, detector_backend='opencv')
            # for x,y,w,h in faces:
            # 			self.img=cv2.rectangle(self.frame_msgs,(x,y),(x+w,y+h),
            # 			(random.randint(0,255),random.randint(0,255),random.randint(0,255)),1)  #making a recentangle to show up and detect the face and setting it position and colour
            # 			self.face_width = h
            print("Face Detected")
            return True
        except:
            print("No face detected")
            return False
        analyze = [DeepFace.analyze(frame,actions=['emotion'], enforce_detection=False)]  #same thing is happing here as the previous example, we are using the analyze class from deepface and using ‘frame’ as input
        emotion = analyze[0]['emotion']
        dominant = analyze[0]['dominant_emotion']
        print(f'Dominant: {dominant} \nEmotions:\n{emotion}')
        # Display the resulting frame
        # cv2.imshow('Camera Input', frame)
        
        # Press 'q' to stop capturing
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
    
    # Release the VideoCapture and VideoWriter objects
    cap.release()
    out.release()
    
    # Destroy all windows
    cv2.destroyAllWindows()
    

save_camera_input_to_file("/home/hooshang/Desktop/OSSRP/cleancoded/src/infrastructure/scripts/basics/image/face_emotion_analysis/vid.mp4")
