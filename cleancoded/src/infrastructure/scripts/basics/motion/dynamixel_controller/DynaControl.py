#!/usr/bin/env python3
"""
Dynamixel Controller for the Dynamixel units used in the robot. 
"""

import argparse
import os
import sys
import time

import rospkg
import rospy
from std_msgs.msg import String
from pyax12.connection import Connection
from infrastructure.msg import DynaTwist, DynaStatus, DynaTwistMultiple

pkg = rospkg.RosPack().get_path('infrastructure')
module_path = os.path.join(pkg, 'scripts', 'tools', 'helper_modules')
sys.path.append(module_path)

from serial_port_handler import Search_for_the_serial_port as port_in_use




class DynaControl:
    """
    Class representing the Dynamixel controller.

    Attributes:
        args (argparse.Namespace): Parsed command line arguments.
        timeout (int): Timeout value for serial communication.
        control_table (list): List of control table entries.
        port (str): Port of the TTL.
        baud_rate (str): Baudrate of the TTL.
        head_id (str): ID of the head Dynamixel unit.
        neck_id (str): ID of the neck Dynamixel unit.
        lhand_id (str): ID of the left hand Dynamixel unit.
        rhand_id (str): ID of the right hand Dynamixel unit.
        connected_ids (list): List of connected Dynamixel IDs.
        serial_connection (Connection): Serial connection object.
        joint_id_lists (list): List of joint-ID pairs.
    """

    rospy.init_node("dynamixel_control", anonymous=False) # initialize the node
    pub = rospy.Publisher('/dyna_status',DynaStatus,queue_size=10) # publish the dynamixel status to the topic
    pub_log = rospy.Publisher('/log/body',String,queue_size=10) # publish the log to the topic


    def __init__(self):
        """
        Initialize the Dynamixel controller.
        """
        
        log_message = f"{rospy.get_caller_id()} Initializing the Dynamixel controller"
        rospy.loginfo(log_message)
        self.pub_log.publish(log_message)

        self.args = self.parse_arguments() # parse the arguments
        self.timeout = 20
        self.control_table = []
        # if the port and baudrate are not given as arguments,
            # the Dynamixel module will scan for the connected Dynamixel units
        if self.args.__getattribute__('port'):
            self.port = self.args.__getattribute__('port')
        else:
             self.port = port_in_use()

        # if the Dynamixel IDs are not given as arguments,
        #    the Dynamixel module will scan for the connected Dynamixel units
        if self.args.__getattribute__('neck') and self.args.__getattribute__('head') and self.args.__getattribute__('lhand') and self.args.__getattribute__('rhand') and self.args.__getattribute__('baudrate'):
            self.baud_rate = self.args.__getattribute__('baudrate')
            self.head_id = self.args.__getattribute__('head')
            self.neck_id = self.args.__getattribute__('neck')
            self.lhand_id = self.args.__getattribute__('lhand')
            self.rhand_id = self.args.__getattribute__('rhand')
            self.connected_ids = [self.head_id, self.neck_id, self.rhand_id, self.lhand_id]

        else:
            # baud_rates = [1000000] # If you don't know the baud rate, DO NOT pass the list as an argument to the scan() function.
                                     # The scan() function has all the possible baud_rates list as a default argument.
                                     # This will make the Dynamixel module scan function try each baud rate to find the connected Dynamixel unit(s).
            self.connected_ids, baud_rate_list = self.scan() # scan for the connected Dynamixel units and the connected baud rate
            self.connected_ids.sort()
            if len(baud_rate_list)>1:
                log_message = f"{rospy.get_caller_id()} More than one baud rate is found. Please specify the baud rate as an argument. Exiting..."
                rospy.logerr(log_message)
                self.pub_log.publish(log_message)
                sys.exit()
            else:
                self.baud_rate = baud_rate_list[0]

        # matching the joint names to the joint's Dynamixel IDs
        self.joint_id_lists = self.match_joint_to_id(
                                        joints=['rhand','neck','head','lhand'], # the sequence of the joints must be the same as the sequence of the IDs
                                        ids=self.connected_ids
                                        )
        try:
            self.serial_connection = Connection(
                port=self.port,
                baudrate=self.baud_rate,
                timeout=self.timeout,
                waiting_time=0.02,
                rpi_gpio=False)

            
            log_message = f"{rospy.get_caller_id()} Connected to the serial port. \nPort: {self.port},\nBaudrate: {self.baud_rate},\nWaiting time: 0.02,\nTimeout: {self.timeout}"
            rospy.loginfo(log_message)
            self.pub_log.publish(log_message)
            
            for joint in self.joint_id_lists: # publish the current position of the joints to the topic
                current = self.currentposition(dynamixel_id=joint[1], degrees=True)
                msg = DynaStatus()
                msg.joint = joint[0]
                msg.position = current
                self.pub.publish(msg)
                
                log_message = f"{rospy.get_caller_id()} {joint[0]} is at position: {current}"
                rospy.loginfo(log_message)
                self.pub_log.publish(log_message)

        except BaseException as e:
            rospy.logerr(e)
            
            log_message = f"{rospy.get_caller_id()} Could not connect to the serial port.\nPort: {self.port},\nBaudrate: {self.baud_rate},\nWaiting time: 0.02,\nTimeout: {self.timeout}\n"
            rospy.logerr(log_message)
            self.pub_log.publish(log_message)
            sys.exit()

        rospy.loginfo(rospy.get_caller_id() + str(self.joint_id_lists))
        rospy.Subscriber("/cmd_vel/dyna", DynaTwist, self.gotodegree) # subscribe to the topic to get the movement commands  
        rospy.Subscriber("/cmd_vel/dyna/multiple", DynaTwistMultiple, self.multiple_commands_callback)   # subscribe to the topic to get the multiple movement commands
        
        log_message = f"{rospy.get_caller_id()} Successfully subscribed to /cmd_vel/dyna and /cmd_vel/dyna/multiple. \n Waiting for commands."
        rospy.loginfo(log_message)
        self.pub_log.publish(log_message)
            

    def parse_arguments(self):
        """
        Parse command line arguments.
        """
        parser = argparse.ArgumentParser(
                description='''If you know the port enter it using --port\n
                            Set Dynamixel IDs assigned to different joints using 
                            --head, --neck, --lhand, --rhand\n''')
        parser.add_argument(
                    '__name', type=str)
        parser.add_argument(
                    '__log', type=str)
        parser.add_argument(
                    '--head', type=int,
                    help='ID (e.g., 1,2,3,etc.)',
                    required=False)
        parser.add_argument(
                    '--neck', type=int,
                    help='ID (e.g., 1,2,3,etc.)',
                    required=False)
        parser.add_argument(
                    '--lhand', type=int,
                    help='ID (e.g., 1,2,3,etc.)',
                    required=False)
        parser.add_argument(
                    '--rhand', type=int,
                    help='ID (e.g., 1,2,3,etc.)',
                    required=False)
        parser.add_argument(
                    '--port', type=str,
                    help='USB port of the TTL (e.g., COM8 (Windows), /dev/ttyUSB0 (Linux))',
                    required=False)
        parser.add_argument(
                    '--baudrate', type=str,
                    help='Baudrate of the TTL (e.g., 1000000)',
                    required=False)

        args = parser.parse_args()
        return args

    def port_in_use(self):
        """
        Search for the serial port in use.
        """
        return port_in_use()

    def scan(self, baud_rates=None):
        """
        Scan for connected Dynamixel units.
        """
        if baud_rates is None:
            baud_rates = [1000000, 9000, 57600, 115200, 2000000, 3000000, 4000000, 4500000]

        connected_baudrates = []  # list of connected baud rates
        ids = []  # list of connected Dynamixel IDs on all baud rates

        # Connect to the serial port
        for baud_rate in baud_rates:
            self.serial_connection = Connection(
                port=self.port,
                baudrate=baud_rate,
                timeout=self.timeout,
                waiting_time=0.02,
                rpi_gpio=False
            )

            rospy.loginfo(rospy.get_caller_id() + ' Connected to the serial port')
            try:
                rospy.loginfo(rospy.get_caller_id() + f' Pinging the dynamixels on baudrate: {baud_rate}')
                # Ping the dynamixel unit(s)
                ids_available = self.serial_connection.scan()  # available ids on the current baud rate

                for dynamixel_id in ids_available:
                    ids.append(dynamixel_id)
                    connected_baudrates.append(baud_rate)
                    self.serial_connection.pretty_print_control_table(dynamixel_id=dynamixel_id)

                rospy.loginfo(rospy.get_caller_id() + f' Scanning for connected Dynamixel units...\n'
                                                      f' Baudrate: {baud_rate}\n'
                                                      f' Found: {ids}')

            except Exception as e:
                print(e)

            self.serial_connection.close()  # close the serial connection
            rospy.loginfo(rospy.get_caller_id() + ' Closed the serial port to try the next baud rate')

        connected_ids = [(f'id: {id}', f'baudrate: {baudr}') for id, baudr in zip(ids, connected_baudrates)]
        
        log_message = f"{rospy.get_caller_id()} Connected Dynamixel Units: {connected_ids}"
        rospy.loginfo(log_message)
        self.pub_log.publish(log_message)

        return ids, connected_baudrates



    def match_joint_to_id(self, joints, ids):
        """
        Match the connected Dynamixel units to the joints' names.
        """
        if len(ids) == len(joints):
            joint_id_lists = [[j, int(i)] for j, i in zip(joints, ids)]
            return joint_id_lists
        
        
        log_message = f"{rospy.get_caller_id()} The number of joints and ids do not match. Please check the arguments. Exiting..."
        rospy.logerr(log_message)
        self.pub_log.publish(log_message)
        sys.exit()

    def gotodegree(self, data, multiple=False):
        """
        Move the Dynamixel units to the specified position in degrees.
        """
        log_message = f'Command recieved: {data}'
        rospy.loginfo(log_message)
        self.pub_log.publish(log_message)

        safety_checked_commands = []
        if multiple:
            commands = data.commands
        else:
            commands = [data]

        command_number = 0
        for command in commands:
            # rospy.sleep(0.5)
            joint = command.joint
            position = command.position
            speed = int(command.speed.angular.x)

            dynamixel_id = 4  # default value for the dynamixel id
            for joint_couple in self.joint_id_lists:  # get the dynamixel id for the requested joint
                if joint == joint_couple[0]:
                    dynamixel_id = joint_couple[1]

            if joint == 'reset':  # if the command is to reset the dynamixel units to their initial position
                log_message = f'{rospy.get_caller_id()} Reseting all the Dynamixel units to the initial position'
                rospy.loginfo(log_message)
                self.pub_log.publish(log_message)
                for dyna in self.joint_id_lists:
                    self.serial_connection.goto(dyna[1], 0, speed=60, degrees=True)
                    time.sleep(0.1)
            else:
                if joint == 'lhand':
                    min_position = -150
                    max_position = 50
                elif joint == 'neck':
                    min_position = -68
                    max_position = 0
                elif joint == 'head':
                    min_position = -150
                    max_position = 150
                elif joint == 'rhand':
                    min_position = -150
                    max_position = 100
                
                # FAIL SAFE: making sure that the respective position for the requested joint is
                # within the factory limits of that specific Dynamixel unit to prevent actuator overloading
                # control_table = self.serial_connection.get_control_table_tuple(dynamixel_id)
                # for d in control_table:
                #     # Set the min and max position to the manually defined min & max positions if they are within the preferred limits,
                #     # otherwise set it to the factory limits
                #     if d[0] == 'cw_angle_limit':
                #         min_position_factory = float(d[1][:4])
                #         min_position = max(min_position, min_position_factory)
                #     if d[0] == 'ccw_angle_limit':
                #         max_position_factory = float(d[1][:3])
                #         max_position = min(max_position, max_position_factory)

                # END OF FAIL SAFE
                if position>max_position or position<min_position:
                    log_message = f"The requested position, {position}, is outside the allowed range of {min_position} to {max_position}.\nSetting the goal position to the closest limit."
                    rospy.loginfo(log_message)
                    self.pub_log.publish(log_message)
                # make sure that the goal position is within the limits
                position = min(max(position, min_position), max_position)
                try:
                    current = self.currentposition(dynamixel_id=dynamixel_id, degrees=True)
                    log_message = f'{rospy.get_caller_id()} Moving dynamixel ID {str(dynamixel_id)} from {current} to position: {str(position)} with speed: {str(speed)}'
                    rospy.loginfo(log_message)
                    self.pub_log.publish(log_message)
                    # move the Dynamixel unit to the requested position
                    self.serial_connection.goto(dynamixel_id, position, speed=speed, degrees=True)
                except Exception as e:
                    log_message = f'{rospy.get_caller_id()} Error from Dynamixel units: {e}. Exiting...'
                    rospy.loginfo(log_message)
                    self.pub_log.publish(log_message)
                    sys.exit()
                    
                




            # While waiting, publish the current position of the joint to the topic.
            # This may cause problems if you need to move couples of multiple joints one after another in a sequence without delay,
            # because it will wait for the first joint/joints couple to finish before moving to the next one.
        # while True:
        #     if not any(self.is_moving_status(dynamixel_id=command[0]) for command in safety_checked_commands):
        #         for command in safety_checked_commands:
        #             dynamixel_id = command[0]
        #             joint = command[1][2]
        #             current = self.currentposition(dynamixel_id=dynamixel_id, degrees=True)
        #             log_message = f"{rospy.get_caller_id()} {joint} stopped at position: {current}"
        #             rospy.loginfo(log_message)
        #             self.pub_log.publish(log_message)

        #         log_message = "All the Dynamixel units have stopped. Waiting for the next command."
        #         rospy.loginfo(log_message)
        #         self.pub_log.publish(log_message)
        #         break
                
        #     time.sleep(0.5)
        #     for command in safety_checked_commands:
        #         try:
        #             dynamixel_id = command[0]
        #             joint = command[1][2]
        #             is_moving = self.is_moving_status(dynamixel_id=dynamixel_id)
        #             if not is_moving:
        #                 current = self.currentposition(dynamixel_id=dynamixel_id, degrees=True)
        #                 msg = DynaStatus()
        #                 msg.joint = joint
        #                 msg.position = current
        #                 self.pub.publish(msg)
                        
        #             else:
        #                 log_message = f"{rospy.get_caller_id()} Waiting for the movement of the {joint} to finish..."
        #                 rospy.loginfo(log_message)
        #                 self.pub_log.publish(log_message)
        #         except:
        #             pass

            


    def multiple_commands_callback(self, data):
        """
        Callback function for multiple commands.
        """
        self.gotodegree(data, multiple=True)

    def currentposition(self, dynamixel_id, degrees=True):
        """
        Get the current position of a Dynamixel unit.
        """
        rospy.loginfo(f"{rospy.get_caller_id()} Getting the current position of dynamixel ID: {dynamixel_id}")
        return self.serial_connection.get_present_position(dynamixel_id, degrees=degrees)

    def is_moving_status(self, dynamixel_id):
        """
        Check if a Dynamixel unit is currently moving.
        """
        rospy.loginfo(f"{rospy.get_caller_id()} Checking if dynamixel ID: {dynamixel_id} is moving")
        return self.serial_connection.is_moving(dynamixel_id)

    def close(self):
        """
        Close the serial connection.
        """
        
        log_message = f"{rospy.get_caller_id()} Closing the serial connection"
        rospy.loginfo(log_message)
        self.pub_log.publish(log_message)
        self.serial_connection.close()
        rospy.loginfo(rospy.get_caller_id()+
                      ' Closed the serial connection')




if __name__ == "__main__": # main function
    camcap = DynaControl() # create an instance of the class
    rospy.spin() # keep the node running until it is stopped
    camcap.close() # close the serial connection
