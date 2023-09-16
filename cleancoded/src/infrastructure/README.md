**List of developed ROS Topics:**
<!-- **** Image Processing **** -->
- /image_raw -> raw image from camera
- /camera_info -> width, height, distortion, etc.
- /image_cv2 -> image converted to cv2
- /image_raw/landmarked -> image with landmarks in ROS Image format
- /image_cv2/landmarked -> image with landmarks in cv2 format
- /gaze_pose -> gaze pose in String format
  
<!-- **** Audio Processing **** -->
- /audio_features -> audio features in custom msg format called 'AudFeatures' (see below)
<!-- - /speech_emotion_analysis -> emotion probabilities in custom msg format called 'EmoProbArr' -->
- /captured_audio -> audio captured from microphone in ROS AudioData format
- /transcript -> transcript of speech in string format


**List of developed ROS Services:**
- audio_features
- speech_emotion_analysis
- speech_to_text
- gaze_pose
<!-- - text_to_speech -->

**List of developed ROS Actions:**
- None -

**List of developed ROS Parameters:**
- None -

**List of developed ROS msg files:**
- AudFeatures.msg
- EmoProbArr.msg
- Landmarks.msg
- List.msg
- Array3D.msg


***List of developed ROS srv files:**
- AudFeature.srv
- EmoProb.srv
- Gaze.srv
- Stt.srv
- Tts.srv

**List of developed ROS action files:**
- None -

**List of developed ROS launch files:**
- init_robot.launch
- initiate_facial.launch
<!-- - gaze_pose.launch
- speech_emotion_analysis.launch
- speech_to_text.launch
- text_to_speech.launch -->

**List of developed ROS config files:**
- None -

**List of developed ROS nodes:**
- opencv_client
- audio_recorder
- speech_emotion_analysis_server
- audio_feature_extractor
- FaceEmotionAnalysis
- landmark_detection
- gaze_detector
- gaze_pose_node
- speech_to_text_server
- text_to_speech_server
- speech_to_text_node