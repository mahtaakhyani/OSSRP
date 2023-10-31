#!/usr/bin/env python3
import cv2
import numpy as np
import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import Float64MultiArray, String
from infrastructure.msg import List, Array3D


class CameraCapture:
    height = 800
    width = 600
    rate = 20
    queue_size = 10
    modif_image = np.zeros((640,480,3))
    rospy.init_node("opencv_client", anonymous=False) # initialize the node 
    pub = rospy.Publisher('/image_cv2',List,queue_size=10) # publish the image to the topic
    


    def __init__(self):
        
        rospy.Subscriber("/camera_info", CameraInfo, self.syncinfo)  
        rospy.Subscriber("/image_raw", Image, self.convert_frame, callback_args=False, queue_size=1, buff_size=2**19) 
        rospy.loginfo("OK")


    def syncinfo(self, info):  # sync camera video stream info
        self.height = info.height
        self.width = info.width
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
            self.modif_image = cv2.resize(frame, (480,640)) # resize the image to the desired size which is the camera info size
            self.modif_image = cv2.cvtColor(self.modif_image, cv2.COLOR_BGR2RGB) # convert the image to RGB
            self.modif_image.flags.writeable =False # To improve performance, optionally mark the image as not writeable to pass by reference.


            te = Array3D()
            tem = List()
            temp = list()
            pub = rospy.Publisher('/testaki',Image,queue_size=10)
            llll=self.modif_image.tolist()
            for i in llll:
                for j in i:
                        obj=Array3D()
                        obj.data=j
                        tem.data.append(obj) # convert the image to a list of 3D arrays 
            rospy.loginfo("Converted the image to a list of 3D arrays")
            cv_bridge=CvBridge()
            arr = []
            l = tem.data

            for i in range(len(l)):
                arr.append(l[i].data) # convert the list of 3D arrays to a list of lists since the 3D array is not supported in ROS

            self.pub.publish(tem)

            ros_image = Image()
            ros_image.header.stamp = rospy.Time.now()
            ros_image.height = cv2_img.shape[0]
            ros_image.width = cv2_img.shape[1]
            ros_image.encoding = "bgr8"
            ros_image.step = cv2_img.shape[1] * 3
            ros_image.data = np.array(cv2_img).tobytes()

            pub.publish(ros_image) # publish the ROS Image message to the topic

            if cv2window:  # whether to show the frames in an opencv window apart from ROS image_view or not
                cv2.imshow("output window", self.modif_image)
            
            rospy.loginfo("Published the image to the topic")
            return self.modif_image
        
        except Exception as e:
            rospy.logerr("Error: %s"%e)
            return e


if __name__ == "__main__": # main function
        camcap = CameraCapture() # create an instance of the class
        rospy.spin() # keep the node running until it is stopped
