#!/usr/bin/env python3
import rospy
from pyax12.connection import Connection
import time
import rospkg
import sys, os
from std_msgs.msg import String
from infrastructure.msg import DynaTwist, DynaStatus
pkg = rospkg.RosPack().get_path('infrastructure')
module_path = os.path.join(pkg, 'scripts', 'tools', 'helper_modules')
sys.path.append(module_path)

from serial_port_handler import Search_for_the_serial_port as port_in_use



class DynaControl:
    rospy.init_node("dynamixel_control", anonymous=False) # initialize the node 
    pub = rospy.Publisher('/dyna_status',DynaStatus,queue_size=10) # publish the dynamixel status to the topic

    def __init__(self):
        # self.connected_ids = self.scan()
        # joints = ['rhand','lhand','neck','head']
        # # position ranges: head>0 , hands: 50<best<-100 , neck: 0<best<150 -> face forward=90 |-> more than 150 is NOT possible at all -> -150|__0__|150
        # default_ids = [3,6,4,1]
        # if len(self.connected_ids)==4:
        #     self.joint_id_lists = [[j,i] for j,i in zip(joints, self.connected_ids)]
        # else:
        #     self.joint_id_lists = [[j,i] for j,i in zip(joints, default_ids)]
        # self.joint_id_lists = [[j,i] for j,i in zip(joints, self.connected_ids)]

        # rospy.loginfo(self.joint_id_lists)
        rospy.Subscriber("/cmd_vel/dyna", DynaTwist, self.gotodegree)  
        rospy.loginfo("Successfully subscribed to /cmd_vel/dyna. \n Waiting for commands.")


    def scan(self):
         # Dynamixel serial port
        dynamixel_serial_port = port_in_use()
        self.baud_rates = [1000000] # If you don't know the baud rate, add 9000, 57600, 115200, 1000000, 
                                        # 2000000, 3000000, 4000000, and 4500000 to this list.
                                        #    This will make the Dynamixel module scan function try each baud rate to find the connected Dynamixel unit(s).
        self.timeout = 20
        self.control_table = []
        connected_baudrates = []
        ids = []
        # Connect to the serial port
        for baud_rate in self.baud_rates:
            self.serial_connection = Connection(
            port=dynamixel_serial_port,
            baudrate=baud_rate, 
            timeout=self.timeout,
            waiting_time=0.02, rpi_gpio=False)
        
            rospy.loginfo('Connected to the serial port')
            try:
                print('pinging the dynamixels')
                # Ping the dynamixel unit(s)
                ids_available = self.serial_connection.scan()
                for dynamixel_id in ids_available:
                    ids.append(dynamixel_id)
                    connected_baudrates.append(baud_rate)
                    self.serial_connection.pretty_print_control_table(dynamixel_id=dynamixel_id)
                

                rospy.loginfo('Scanning for connected Dynamixel units...','\n','Baudrate:',baud_rate,'Found:',ids)
            except BaseException as e:
                print(e)
        
        connected_ids = [('id: %s'%id,'baudrate: %s'%baudr) for id,baudr in zip(ids, connected_baudrates)]
        rospy.loginfo('Connected Dynamixel Units: {}'.format(connected_ids)) 
        return ids


    def gotodegree(self, data):
        rospy.loginfo("recieved a movement command")
        speed = int(data.speed.angular.x) # the speed must be given via the angular velocity of the Twist msg (NOT linear) and necessarily for the roll rotation
        dposition = int(data.position) # the requested movement value (aka. how much the joint should move from its current position)
        degrees=True # the requested position value is considered in degrees
        req_joint = data.joint # the requested joint's name (head, neck, rhand, or lhand)
        # Check which id is for the requested joint
        dynamixel_id = 4
        for joint in self.joint_id_lists:
            if joint[0] == req_joint:
                dynamixel_id = joint[1]
        # try:
        current = self.currentposition(dynamixel_id, degrees=degrees)

        # get the preferred limits for the requested joint (the limits are based on the robot's anatomy and can be changed if needed)
        if req_joint== 'head':
                min_position = 0
                max_position = 150
        if req_joint== 'neck':
                min_position = 0
                max_position = 150
        if req_joint== 'rhand':
                min_position = -100
                max_position = 50
        if req_joint== 'lhand':
                min_position = -100
                max_position = 50

        # FAIL SAFE: making sure that the respective position for the requested joint is within the factory limits of that specific Dynamixel unit to prevent overloading
        control_table = self.serial_connection.get_control_table_tuple(dynamixel_id)
        for d in control_table:
            if d[0] == 'cw_angle_limit':
                min_position_factory = float(d[1][:4])
            if d[0] == 'ccw_angle_limit':    
                max_position_factory = float(d[1][:3])

            if min_position<min_position_factory:
                min_position = min_position_factory
            if max_position>max_position_factory:
                max_position = max_position_factory
        # END OF FAIL SAFE

        # Set the goal position to the requested position if it is within the preferred limits, otherwise set it to the preferred limits
        if min_position < current+dposition < max_position:
            goal_position = current+dposition
        elif current+dposition < min_position:
            goal_position = min_position
        else:
            goal_position = max_position

        rospy.loginfo('Moving to position: ', dposition)
        # Go to the specified position in degrees
        self.serial_connection.goto(dynamixel_id, goal_position, speed=speed, degrees=degrees)

        rospy.loginfo('Waiting for movement to finish...')
        while True:
            time.sleep(0.1)
            if self.is_moving_status(dynamixel_id=dynamixel_id):
                # current = self.currentposition(dynamixel_id, degrees=degrees)
                # msg = DynaStatus()
                # msg.joint = req_joint
                # msg.position = current
                # self.pub.publish(msg)
                break
            else: pass
        # except BaseException as e:
        #     rospy.logerr(e)


    def currentposition(self, dynamixel_id, degrees=True):
        # Get the current position
        current_position = self.serial_connection.get_present_position(dynamixel_id, degrees=degrees)
        rospy.loginfo('Current position: ', current_position,'\n')
        return current_position

    def is_moving_status(self, dynamixel_id):
        is_moving = self.serial_connection.get_control_table_tuple(dynamixel_id)[-3][1]
        if is_moving == 'no':
                rospy.loginfo( 'Movement finished. Stopping...')
                return True
        else:
            rospy.loginfo('Still moving...')
            return False
    


if __name__ == "__main__": # main function
        camcap = DynaControl() # create an instance of the class
        rospy.spin() # keep the node running until it is stopped
