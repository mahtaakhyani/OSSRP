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
from pyax12.connection import Connection
from std_msgs.msg import String
from infrastructure.msg import DynaTwist, DynaStatus, DynaTwistMultiple
from serial_port_handler import Search_for_the_serial_port as port_in_use

pkg = rospkg.RosPack().get_path('infrastructure')
module_path = os.path.join(pkg, 'scripts', 'tools', 'helper_modules')
sys.path.append(module_path)

from serial_port_handler import Search_for_the_serial_port as port_in_use
import rospy
import sys
import os
import time
import argparse
from std_msgs.msg import String
from infrastructure.msg import DynaTwist, DynaStatus, DynaTwistMultiple
from pyax12.connection import Connection
from serial_port_handler import Search_for_the_serial_port as port_in_use
import rospkg




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

    def __init__(self):
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
        if self.__getattribute__('neck') and self.__getattribute__('head') and self.__getattribute__('lhand') and self.__getattribute__('rhand') and self.__getattribute__('baudrate'):
            self.baud_rate = self.args.__getattribute__('baudrate')
            self.head_id = self.args.__getattribute__('head')
            self.neck_id = self.args.__getattribute__('neck')
            self.lhand_id = self.args.__getattribute__('lhand')
            self.rhand_id = self.args.__getattribute__('rhand')
            self.connected_ids = [self.head_id, self.neck_id, self.lhand_id, self.rhand_id]

        else:
            # baud_rates = [1000000] # If you don't know the baud rate, DO NOT pass the list as an argument to the scan() function.
                                     # The scan() function has all the possible baud_rates list as a default argument.
                                     # This will make the Dynamixel module scan function try each baud rate to find the connected Dynamixel unit(s).
            self.connected_ids, baud_rate_list = self.scan() # scan for the connected Dynamixel units and the connected baud rate
            self.joint_id_lists = self.match_joint_to_id(
                                        joints=['lhand', 'neck', 'head', 'rhand'], # the sequence of the joints must be the same as the sequence of the IDs
                                        ids=self.connected_ids
                                        )
            
            if len(baud_rate_list)>1:
                rospy.logerr(rospy.get_caller_id()+
                             ' More than one baud rate is found. Please specify the baud rate as an argument. Exiting...')
                sys.exit()
            else:
                self.baud_rate = baud_rate_list[0]
            
        try:
            self.serial_connection = Connection(
                port=self.port,
                baudrate=self.baud_rate,
                timeout=self.timeout,
                waiting_time=0.02,
                rpi_gpio=False)

            rospy.loginfo(f"{rospy.get_caller_id()} Connected to the serial port. "
                            f"Port: {self.port},\n Baudrate: {self.baud_rate},\n "
                            f"Waiting time: 0.02, Timeout: {self.timeout}\n")
            
            for joint in self.joint_id_lists: # publish the current position of the joints to the topic
                current = self.currentposition(dynamixel_id=joint[1], degrees=True)
                msg = DynaStatus()
                msg.joint = joint[0]
                msg.position = current
                self.pub.publish(msg)
                log_message = f"{rospy.get_caller_id()} {joint[0]} is at position: {current}"
                rospy.loginfo(log_message)

        except BaseException as e:
            rospy.logerr(e)
            rospy.logerr(f"{rospy.get_caller_id()} Could not connect to the serial port. "
                            f"Port: {self.port},\n Baudrate: {self.baud_rate},\n "
                            f"Waiting time: 0.02, Timeout: {self.timeout}\n")
            sys.exit()

        rospy.loginfo(rospy.get_caller_id() + str(self.joint_id_lists))
        rospy.Subscriber("/cmd_vel/dyna", DynaTwist, self.gotodegree) # subscribe to the topic to get the movement commands  
        rospy.Subscriber("/cmd_vel/dyna/multiple", DynaTwistMultiple, self.multiple_commands_callback)   # subscribe to the topic to get the multiple movement commands
        rospy.loginfo(rospy.get_caller_id()+
                      " Successfully subscribed to /cmd_vel/dyna and /cmd_vel/dyna/multiple. \n Waiting for commands.")

            

    def parse_arguments(self):
        """
        Parse command line arguments.
        """
        parser = argparse.ArgumentParser(
                description='''If you know the port enter it using --port\n
                            Set Dynamixel IDs assigned to different joints using 
                            --head, --neck, --lhand, --rhand\n''')
        parser.add_argument(
                    '--head', type=str,
                    help='ID (e.g., 1,2,3,etc.)',
                    required=False)
        parser.add_argument(
                    '--neck', type=str,
                    help='ID (e.g., 1,2,3,etc.)',
                    required=False)
        parser.add_argument(
                    '--lhand', type=str,
                    help='ID (e.g., 1,2,3,etc.)',
                    required=False)
        parser.add_argument(
                    '--rhand', type=str,
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
        rospy.loginfo(rospy.get_caller_id() + f' Connected Dynamixel Units: {connected_ids}')

        return ids, connected_baudrates



    def match_joint_to_id(self, joints, ids):
        """
        Match the connected Dynamixel units to the requested joints.
        """
        if len(ids) == len(joints):
            joint_id_lists = [[j, i] for j, i in zip(joints, ids)]
            return joint_id_lists
        rospy.logerr(rospy.get_caller_id() + ' The number of joints and ids do not match. Please check the arguments.')


    def gotodegree(self, data, multiple=False):
        """
        Move the Dynamixel units to the specified position in degrees.
        """
        safety_checked_commands = []
        if multiple:
            commands = data.commands
        else:
            commands = [data]

        command_number = 0
        for command in commands:
            joint = command.joint
            position = command.position
            speed = int(command.speed.angular.x)

            dynamixel_id = 4  # default value for the dynamixel id
            for joint_couple in self.joint_id_lists:  # get the dynamixel id for the requested joint
                if joint == joint_couple[0]:
                    dynamixel_id = joint[1]
            safety_checked_commands.append([dynamixel_id])

            if joint == 'reset':  # if the command is to reset the dynamixel units to their initial position
                for dyna in self.joint_id_lists:
                    self.serial_connection.goto(dyna[1], 0, speed=100, degrees=True)
                    time.sleep(0.1)
            else:
                if joint == 'lhand':
                    min_position = -150
                    max_position = 40
                elif joint == 'neck':
                    min_position = 0
                    max_position = 150
                elif joint == 'head':
                    min_position = 0
                    max_position = 150
                elif joint == 'rhand':
                    min_position = -150
                    max_position = 40

                # FAIL SAFE: making sure that the respective position for the requested joint is
                # within the factory limits of that specific Dynamixel unit to prevent actuator overloading
                control_table = self.serial_connection.get_control_table_tuple(dynamixel_id)
                for d in control_table:
                    # Set the min and max position to the manually defined min & max positions if they are within the preferred limits,
                    # otherwise set it to the factory limits
                    if d[0] == 'cw_angle_limit':
                        min_position_factory = float(d[1][:4])
                        min_position = max(min_position, min_position_factory)
                    if d[0] == 'ccw_angle_limit':
                        max_position_factory = float(d[1][:3])
                        max_position = min(max_position, max_position_factory)

                # END OF FAIL SAFE

                # make sure that the goal position is within the limits
                position = min(max(position, min_position), max_position)
                safety_checked_commands[command_number].append([position,speed,joint])
                command_number+=1

        # Now that we have gathered all the safety checked commands, we can move the Dynamixel units at the same time
        for command in safety_checked_commands:
            dynamixel_id = command[0]
            goal_position = command[1][0]
            speed = command[1][1]
            current = self.currentposition(dynamixel_id=dynamixel_id, degrees=True)
            log_message = rospy.get_caller_id() + f' Moving dynamixel ID {str(dynamixel_id)} from {current} to position: {str(goal_position)} with speed: {str(speed)}'
            rospy.loginfo(log_message)
            # move the Dynamixel unit to the requested position
            self.serial_connection.goto(dynamixel_id, goal_position, speed=speed, degrees=True)


            # While waiting, publish the current position of the joint to the topic.
            # This may cause problems if you need to move couples of multiple joints one after another in a sequence without delay,
            # because it will wait for the first joint/joints couple to finish before moving to the next one.
            while True:
                if not any(self.is_moving_status(dynamixel_id=command[0]) for command in safety_checked_commands):
                    break
                time.sleep(0.2)
                for command in safety_checked_commands:
                    dynamixel_id = command[0]
                    joint = command[1][2]
                    is_moving = self.is_moving_status(dynamixel_id=dynamixel_id)
                    if not is_moving:
                        current = self.currentposition(dynamixel_id=dynamixel_id, degrees=True)
                        msg = DynaStatus()
                        msg.joint = joint
                        msg.position = current
                        self.pub.publish(msg)
                        log_message = f"{rospy.get_caller_id()} {joint} stopped at position: {current}"
                        rospy.loginfo(log_message)
                    else:
                        log_message = f"{rospy.get_caller_id()} Waiting for the movement of the {joint} to finish..."
                        rospy.loginfo(log_message)


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
        rospy.loginfo(rospy.get_caller_id()+
                      ' Closing the serial connection')
        self.serial_connection.close()
        rospy.loginfo(rospy.get_caller_id()+
                      ' Closed the serial connection')




if __name__ == "__main__": # main function
        camcap = DynaControl() # create an instance of the class
        rospy.spin() # keep the node running until it is stopped
        camcap.close() # close the serial connection
