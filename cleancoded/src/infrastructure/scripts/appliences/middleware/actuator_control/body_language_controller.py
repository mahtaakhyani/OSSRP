#!/usr/bin/env python3
import rospy
from infrastructure.msg import FaceEmotions, EmoProbArr, DynaTwistMultiple, DynaTwist
import time
import argparse

class FeelingsController:
    count = 0
    rospy.init_node("body_language", anonymous=False)
    pub = rospy.Publisher('/cmd_vel/dyna/multiple',DynaTwistMultiple,queue_size=10) # publish the body language to the topic

    def __init__(self, topic_name):
        rospy.Subscriber(topic_name, FaceEmotions, self.feeling_callback)
        rospy.loginfo("Successfully subscribed to {}. \n Initializing body language controller...".format(topic_name))
        self.dominant = ""
        msg = DynaTwistMultiple()
        sub_msg = DynaTwist()
        sub_msg.joint = 'reset'
        sub_msg.speed.angular.x = 100
        sub_msg.position = 0
        msg.commands = [sub_msg]
        self.pub.publish(msg) # reset the dynamixels to the initial position

        # position ranges: head>0 , hands: 50<best<-100 , neck: 0<best<150 -> face forward=90 |-> more than 150 is NOT possible at all -> -150|__0__|150
        sub_msg.joint = "lhand"
        sub_msg.speed.angular.x = 100
        sub_msg.position = -25
        msg.commands.append(sub_msg)
        sub_msg.joint = "rhand"
        sub_msg.speed.angular.x = 100
        sub_msg.position = -25
        msg.commands.append(sub_msg)
        sub_msg.joint = "head"
        sub_msg.speed.angular.x = 70
        sub_msg.position = 75
        msg.commands.append(sub_msg)
        sub_msg.joint = "neck"
        sub_msg.speed.angular.x = 70
        sub_msg.position = 90
        self.pub.publish(msg) # moving to more natural position
        


    def feeling_callback(self, data):
        self.count+=1
        if self.count%10 == 0: # slow down the rate of publishing to let the robot react. This is necessary because the robot is not fast enough to react to every single emotion.
            # If you wish to change the rate, change the rate in the auto_exp_node.py file as well. The rate in the auto_exp_node.py file should be the same as the rate in the body_language_controller.py file.
            rospy.loginfo("Received face emotion data")
            # call the determine_feeling function
            self.determine_feeling(data.data)
    

    def determine_feeling(self, data_list):
        # get the emotion with the highest probability
        highest_prob = 0
        for i in range(len(data_list)):
            if data_list[i].probability > highest_prob:
                highest_prob = data_list[i].probability
                feeling = data_list[i].emotion

        if feeling != self.dominant: # to prevent the robot from repeating the same movement
            self.dominant = feeling
            rospy.log_info("feeling is ", self.dominant)
            # call the determine_movement function
            self.determine_movement(self.dominant)
        else:
            rospy.log_info("feeling is still ", self.dominant)
        
    

    def determine_movement(self, feeling):
        # Determine the probable movement of hands and head based on the feeling
        if feeling == "surprised":
        # hand_movement = "raising hands surprisedly"
        # head_movement = "tilting back"

            # raise hands surprisedly:
            position = -100
            joint1 = "lhand"
            joint2 = "rhand"
            speed = 110
            sub_speed = speed/2
            sub_position = position+10
            sub_sub_position = position-10
            hand_action = [[speed, position, joint1], [speed, position, joint2]]
            # a little wave above the head to express the feeling:
            hand_sub_action = [[sub_speed, sub_position, joint1], [sub_speed, sub_position, joint2]]
            hand_sub_sub_action = [[sub_speed, sub_sub_position, joint1], [sub_speed, sub_sub_position, joint2]]

            # tilt head back:
            position = 100
            speed = 100
            joint = "head"
            head_action = [[speed, position, joint]]
            sub_speed = speed
            sub_position = position+30
            sub_sub_position = position-30
            
            head_sub_action = [[sub_speed, sub_position, joint]]
            head_sub_sub_action = [[sub_speed, sub_sub_position, joint]]

            combined_actions = [hand_action, head_action]
            self.actuator_control(combined_actions) # raise hands happily and nod head

            for i in range(3):
                combined_actions = [hand_sub_action, head_sub_action]
                self.actuator_control(combined_actions) # a little wave above the head and nod head
                combined_actions = [hand_sub_sub_action, head_sub_sub_action]
                self.actuator_control(combined_actions) # a little wave above the head and nod head




        elif feeling == "happy":
            # hand_movement = "raising hands happily"
            # head_movement = "nodding"

            # raise hands happily:
            position = -100
            joint1 = "lhand"
            joint2 = "rhand"
            speed = 100
            sub_speed = speed/2
            sub_position = position+10
            sub_sub_position = position-10
            hand_action = [[speed, position, joint1], [speed, position, joint2]]
            # a little wave above the head to express the feeling:
            hand_sub_action = [[sub_speed, sub_position, joint1], [sub_speed, sub_position, joint2]]
            hand_sub_sub_action = [[sub_speed, sub_sub_position, joint1], [sub_speed, sub_sub_position, joint2]]

            # nod head:
            position = 100
            speed = 70
            joint = "head"
            head_action = [[speed, position, joint]]
            sub_speed = speed
            sub_position = position+30
            sub_sub_position = position-30
            
            head_sub_action = [[sub_speed, sub_position, joint]]
            head_sub_sub_action = [[sub_speed, sub_sub_position, joint]]

            combined_actions = [hand_action, head_action]
            self.actuator_control(combined_actions) # raise hands happily and nod head

            for i in range(3):
                combined_actions = [hand_sub_action, head_sub_action]
                self.actuator_control(combined_actions) # a little wave above the head and nod head
                combined_actions = [hand_sub_sub_action, head_sub_sub_action]
                self.actuator_control(combined_actions) # a little wave above the head and nod head




        elif feeling == "sad":
        # hand_movement = "hanging down"
        # head_movement = "head dropping and shaking"

            # hang hands down:
            position = 70
            speed = 80
            joint1 = "lhand"
            joint2 = "rhand"
            hand_action = [[speed, position, joint1], [speed, position, joint2]]

            # head dropping and shaking:
            position = 20
            speed = 60
            joint1 = "neck"
            joint2 = "head"
            joint2_position = 90
            head_action = [[speed, position, joint1], [speed, joint2_position, joint2]]
            sub_speed = speed
            sub_joint2_position = joint2_position+30
            sub_sub_joint2_position = joint2_position-30

            head_sub_action = [[sub_speed, sub_joint2_position, joint2]]
            head_sub_sub_action = [[sub_speed, sub_sub_joint2_position, joint2]]

            combined_actions = [hand_action, head_action]
            self.actuator_control(combined_actions) # raise hands fearfully and look around rapidly

            for i in range(3):
                combined_actions = [head_sub_action]
                self.actuator_control(combined_actions)
                combined_actions = [head_sub_sub_action]
                self.actuator_control(combined_actions) # a little wave above the head and look around rapidly




        elif feeling == "fear":
        # hand_movement = "raising hands fearfully"
        # head_movement = "looking around rapidly"

            # raise hands fearfully:
            position = -100
            joint1 = "lhand"
            joint2 = "rhand"
            speed = 120
            sub_speed = speed/2
            sub_position = position+10
            sub_sub_position = position-10
            hand_action = [[speed, position, joint1], [speed, position, joint2]]
            # a little wave above the head to express the feeling:
            hand_sub_action = [[sub_speed, sub_position, joint1], [sub_speed, sub_position, joint2]]
            hand_sub_sub_action = [[sub_speed, sub_sub_position, joint1], [sub_speed, sub_sub_position, joint2]]

        # look around rapidly:
            position = 90
            speed = 100
            joint = "neck"
            head_action = [[speed, position, joint]]
            sub_speed = speed
            sub_position = position+40
            sub_sub_position = position-40

            head_sub_action = [[sub_speed, sub_position, joint]]
            head_sub_sub_action = [[sub_speed, sub_sub_position, joint]]

            combined_actions = [hand_action, head_action]
            self.actuator_control(combined_actions) # raise hands fearfully and look around rapidly

            for i in range(3):
                combined_actions = [hand_sub_action, head_sub_action]
                self.actuator_control(combined_actions)
                combined_actions = [hand_sub_sub_action, head_sub_sub_action]
                self.actuator_control(combined_actions) # a little wave above the head and look around rapidly




        elif feeling == "disgust":
        # hand_movement = "waving hands away"
        # head_movement = "shaking head"

            # wave hands away:
            position = 70
            speed = 80
            joint1 = "lhand"
            joint2 = "rhand"
            hand_action = [[speed, position, joint1], [speed, position, joint2]]

            # shake head:
            position = 90
            speed = 80
            joint = "neck"
            head_action = [[speed, position, joint]]
            sub_speed = speed
            sub_position = position+40
            sub_sub_position = position-40

            head_sub_action = [[sub_speed, sub_position, joint]]
            head_sub_sub_action = [[sub_speed, sub_sub_position, joint]]

            combined_actions = [hand_action, head_action]
            self.actuator_control(combined_actions) # raise hands fearfully and look around rapidly

            for i in range(3):
                combined_actions = [head_sub_action]
                self.actuator_control(combined_actions)
                combined_actions = [head_sub_sub_action]
                self.actuator_control(combined_actions) # a little wave above the head and look around rapidly




        elif feeling == "angry":
        # hand_movement = "gesturing aggressively"
        # head_movement = "shaking head"
        
        # gesture aggressively:
            lposition = -50
            rposition = 50
            speed = 80
            joint1 = "lhand"
            joint2 = "rhand"
            hand_action = [[speed, lposition, joint1], [speed, rposition, joint2]]
            sub_speed = speed/2
            sub_position = lposition+20
            sub_sub_position = lposition-20
            # one hand gestures to express the feeling:
            hand_sub_action = [[sub_speed, sub_position, joint1]]
            hand_sub_sub_action = [[sub_speed, sub_sub_position, joint1]]

        # shake head:
            position = 90
            speed = 60
            joint = "neck"
            head_action = [[speed, position, joint]]
            sub_speed = speed
            sub_position = position+20
            sub_sub_position = position-20

            head_sub_action = [[sub_speed, sub_position, joint]]
            head_sub_sub_action = [[sub_speed, sub_sub_position, joint]]

            combined_actions = [hand_action, head_action]
            self.actuator_control(combined_actions) # raise hands fearfully and look around rapidly

            for i in range(3):
                combined_actions = [hand_sub_action, head_sub_action]
                self.actuator_control(combined_actions)
                combined_actions = [hand_sub_sub_action, head_sub_sub_action]
                self.actuator_control(combined_actions) # a little wave above the head and look around rapidly


        else: # if feeling == "neutral": move to a more natural position
            joint = "lhand"
            speed = 100
            position = -25
            lhand_action = [[speed, position, joint]]
            joint = "rhand"
            rhand_action = [[speed, position, joint]]
            joint = "head"
            speed = 70
            position = 75
            head_action = [[speed, position, joint]]
            joint = "neck"
            position = 90
            neck_action = [[speed, position, joint]]
            combined_actions = [lhand_action, rhand_action, head_action, neck_action]
            self.actuator_control(combined_actions)



    def actuator_control(self, action):
        # publish the definition of each movement of hands and head to the topic /cmd_vel/dyna/multiple
        msg = DynaTwistMultiple()
        l = [DynaTwist() for i in range(len(action))]
        for i in range(len(action)):
            l[i].joint = action[i][0][2]
            l[i].speed.angular.x = action[i][0][0]
            l[i].position = action[i][0][1]
            msg.commands.append(l[i])
        self.pub.publish(msg)
        time.sleep(0.05)
    

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Subscribe to a ROS topic to get the emotion data used in body language controller')
    parser.add_argument('topic_name', type=str, default='/face_emotions', help='Name of the topic to subscribe to')
    args = parser.parse_args()

    try:
        FeelingsController(args.topic_name)
        rospy.spin()
    except rospy.ROSInterruptException:
        pass