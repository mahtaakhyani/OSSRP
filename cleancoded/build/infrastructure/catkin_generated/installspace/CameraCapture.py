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
    
        cv_bridge = CvBridge()
        try:
            frame_in_cv2 = cv_bridge.imgmsg_to_cv2(data, desired_encoding="passthrough") 
            new_a_shape = (-1,) + frame_in_cv2.shape[2:] # to get rid of the first dimension
            b = np.split(frame_in_cv2.reshape(new_a_shape), frame_in_cv2.shape[0]) 
            frame = np.array(b)

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

            
            arrrrr= np.array(arr, dtype=np.uint8).reshape((640,480,3)) # convert the list of lists to a numpy array
            frame_in_ros = cv_bridge.cv2_to_imgmsg(arrrrr) # convert the numpy array to a ROS Image message
            frame_in_ros.encoding = "rgb8" # set the encoding of the ROS Image message to rgb8

            msg = Image() # create a new ROS Image message
            msg = frame_in_ros # assign the ROS Image message to the new ROS Image message we created
            pub.publish(msg) # publish the ROS Image message to the topic

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
