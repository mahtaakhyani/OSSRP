#!/usr/bin/env python3
import rospy
from infrastructure.msg import FaceEmotions, Exp, DynaTwistMultiple, DynaTwist
from std_msgs.msg import String
import time
import argparse

class FeelingsController:
    count = 0
    rospy.init_node("body_language", anonymous=False)
    pub = rospy.Publisher('/cmd_vel/dyna/multiple',DynaTwistMultiple,queue_size=10) # publish the body language to the topic
    pub_log = rospy.Publisher('/log/body/behaviors',String,queue_size=10) # publish the log to the topic

    def __init__(self):
        self.dominant = ""
        # parse the arguments
        topic_name, msg_type = self.parse_arguments()
        if msg_type == "Exp":
            rospy.Subscriber(topic_name, Exp, self.feeling_exp_callback)
        elif msg_type == "FaceEmotions":
            rospy.Subscriber(topic_name, FaceEmotions, self.feeling_emotionmsg_callback)
        rospy.loginfo(rospy.get_caller_id()+
                      f" Successfully subscribed to {topic_name}. \n Initializing body language controller...")
        

        # reset the position of the dynamixels
        # rospy.sleep(4)
        # self.reset_position()
        # rospy.sleep(1)
        # moving to more natural positions
        # position ranges: head>0 , hands: 50<best<-100 , neck: 0<best<-120 -> face forward=-60 |-> more than 150 or less than -150 is NOT possible
        # self.move_to_more_natural_position()
        
        rospy.loginfo(rospy.get_caller_id()+" Body language controller initialized")

    def parse_arguments(self):
        parser = argparse.ArgumentParser(description='Subscribe to a ROS topic with the specified msg type to get the emotion data used in body language controller')
        parser.add_argument(
            '__name', type=str)
        parser.add_argument(
                    '__log', type=str)
        parser.add_argument('--topic',
                            type=str,
                            help='Name of the topic to subscribe to for emotion data',
                            required=True)
        parser.add_argument('--msg_type',
                            type=str,
                            help='Type of the emotion data input (e.g String or FaceEmotions)',
                            required=True)
            
        args = parser.parse_args()
        topic = args.__getattribute__('topic')
        msg_type = args.__getattribute__('msg_type')
        return topic, msg_type


    # callback function for face emotion data
    def feeling_emotionmsg_callback(self, data):
        delay = 0.5 # Delay for 0.5 seconds
        rospy.sleep(delay)
         # slow down the rate of publishing to let the robot react.
        # This is necessary because the robot is not fast enough to react to every single emotion.
        # If you wish to change the rate, change the rate in the auto_exp_node.py file as well.
        # The rate in the auto_exp_node.py file should be the same as the rate in the body_language_controller.py file.
        log_message = f"{rospy.get_caller_id()} Received face emotion msg with the set {delay} seconds delay."
        rospy.loginfo(log_message)
        self.pub_log.publish(log_message)
        # call the determine_feeling function
        self.determine_feeling(data.data)


# function for extracting the dominant emotion from the emotion data (the emotion with the highest probability from the FaceEmotions messages)
    def determine_feeling(self, data_list):
        # get the emotion with the highest probability
        highest_prob = 0
        for _, data in enumerate(data_list):
            if data.probability > highest_prob:
                highest_prob = data.probability
                feeling = data.emotion

        if feeling != self.dominant: # to prevent the robot from repeating the same movement
            self.dominant = feeling
            
            log_message = f"{rospy.get_caller_id()} feeling is {self.dominant}"
            rospy.log_info(log_message)
            self.pub_log.publish(log_message)
            # call the determine_movement function
            self.determine_movement(self.dominant)
        else:
            
            log_message = f"{rospy.get_caller_id()} feeling hasn't changed and is still {self.dominant}.\nNot reacting to the same feeling twice."
            rospy.log_info(log_message)
            self.pub_log.publish(log_message)

    # callback function for manual emotion input
    def feeling_exp_callback(self, data):
        """
        This function is for manual emotion input for body language control.
        It is used when you want the robot to express a certain emotion.
        """
        
        log_message = f"{rospy.get_caller_id()} Received emotion Exp msg type."
        rospy.loginfo(log_message)
        self.pub_log.publish(log_message)
        self.determine_movement(data.emotion)


    def determine_movement(self, feeling):
        """
        Determine the movement of hands and head and neck based on the given feeling.
        """
        
        self.pub_log.publish(f"{rospy.get_caller_id()} performing {feeling} movement")

        try:
            if "surprise" in feeling:
                self.perform_surprised_movement()

            elif "happy" in feeling:
                self.perform_happy_movement()

            elif "sad" in feeling:
                self.perform_sad_movement()

            elif "fear" in feeling:
                self.perform_fearful_movement()

            elif "disgust" in feeling:
                self.perform_disgusted_movement()

            elif "angry" in feeling:
                self.perform_angry_movement()

            elif feeling == "neutral":
                self.move_to_more_natural_position()
        
        except Exception as e:
            
            log_message = f"{rospy.get_caller_id()} Error: {e}. Resetting position and moving to more natural position."
            rospy.logerr(log_message)
            self.pub_log.publish(log_message)
            self.reset_position()
            self.move_to_more_natural_position()


    def perform_surprised_movement(self):
        """
        Perform the movement for the "surprised" feeling.
        """
        rospy.loginfo(rospy.get_caller_id()+" performing surprised movement")
        # Raise hands surprisedly
        hand_actions_list = self.raise_hands_surprisedly()
        # Tilt head back and nod
        head_actions_list = self.tilt_head_back_surprised()

        # combine the hand and head actions list decussatedly
        all_actions_list = [val for pair in zip(hand_actions_list, head_actions_list) for val in pair]

        # perform the movement
        self.perform_movement(all_actions_list)

    

    def perform_happy_movement(self):
        """
        Perform the movement for the "happy" feeling.
        """
        rospy.loginfo(rospy.get_caller_id()+" performing happy movement")
        # Raise hands happily
        hand_actions_list = self.raise_hands_happily()
        # Tilt head back and nod
        head_actions_list = self.tilt_head_back_happily()

         # combine the hand and head actions list decussatedly
        all_actions_list = [val for pair in zip(hand_actions_list, head_actions_list) for val in pair]

        # perform the movement
        self.perform_movement(all_actions_list)



    def perform_sad_movement(self):
        """
        Perform the movement for the "sad" feeling.
        """
        rospy.loginfo(rospy.get_caller_id()+" performing sad movement")
        # Hang hands down
        hand_actions_list = self.hang_hands_down_sadly()
        # Drop head and shake
        head_actions_list = self.drop_head_and_shake_sadly()

         # combine the hand and head actions list decussatedly
        all_actions_list = [val for pair in zip(hand_actions_list, head_actions_list) for val in pair]

        # perform the movement
        self.perform_movement(all_actions_list)




    def perform_fearful_movement(self):
        """
        Perform the movement for the "fearful" feeling.
        """
        rospy.loginfo(rospy.get_caller_id()+" performing fearful movement")
        # Raise hands fearfully
        hand_actions_list = self.wave_hands_away_fearfully
        # Look around rapidly
        head_actions_list = self.look_around_rapidly_fearfully()

         # combine the hand and head actions list decussatedly
        all_actions_list = [val for pair in zip(hand_actions_list, head_actions_list) for val in pair]

        # perform the movement
        self.perform_movement(all_actions_list)




    def perform_disgusted_movement(self):
        """
        Perform the movement for the "disgusted" feeling.
        """
        rospy.loginfo(rospy.get_caller_id()+" performing disgusted movement")
        # Wave hands away
        hand_actions_list = self.wave_hands_away_disgustedly()
        # Shake head
        head_actions_list = self.shake_head_disgustedly()

         # combine the hand and head actions list decussatedly
        all_actions_list = [val for pair in zip(hand_actions_list, head_actions_list) for val in pair]

        # perform the movement
        self.perform_movement(all_actions_list)



    
    def perform_angry_movement(self):
        """
        Perform the movement for the "angry" feeling.
        """
        rospy.loginfo(rospy.get_caller_id()+" performing angry movement")
        # Gesture aggressively
        hand_actions_list = self.wave_hands_away_agressively()
        # Shake head
        head_actions_list = self.shake_head_aggresively()

         # combine the hand and head actions list decussatedly
        all_actions_list = [val for pair in zip(hand_actions_list, head_actions_list) for val in pair]

        # perform the movement
        self.perform_movement(all_actions_list)


    

    def raise_hands_surprisedly(self):
        rospy.loginfo(rospy.get_caller_id()+" raising hands surprisedly")
        self.pub_log.publish(rospy.get_caller_id()+" raising hands surprisedly")
        # raise hands surprisedly:
        position = -100
        joint1 = "lhand"
        joint2 = "rhand"
        speed = 110
        sub_speed = speed / 2
        sub_position = position + 10
        sub_sub_position = position - 10
        hand_action = [[speed, position, joint1], [speed, position, joint2]]
        # a little wave above the head to express the feeling:
        hand_sub_action = [[sub_speed, sub_position, joint1], [sub_speed, sub_position, joint2]]
        hand_sub_sub_action = [[sub_speed, sub_sub_position, joint1], [sub_speed, sub_sub_position, joint2]]

        combined_hand_actions = [hand_action, hand_sub_action, hand_sub_sub_action]
        return combined_hand_actions



    def tilt_head_back_surprised(self):
        rospy.loginfo(rospy.get_caller_id()+" tilting head back surprised and nodding")
        self.pub_log.publish(rospy.get_caller_id()+" tilting head back surprised and nodding")
        # tilt head back and nod:
        position = 100
        speed = 100
        joint = "head"
        head_action = [[speed, position, joint]]
        sub_speed = speed
        sub_position = position + 30
        sub_sub_position = position - 30
        # nod head:
        head_sub_action = [[sub_speed, sub_position, joint]]
        head_sub_sub_action = [[sub_speed, sub_sub_position, joint]]

        combined_head_actions = [head_action, head_sub_action, head_sub_sub_action]
        return combined_head_actions


    def raise_hands_happily(self):
        rospy.loginfo(rospy.get_caller_id()+" raising hands happily")
        self.pub_log.publish(rospy.get_caller_id()+" raising hands happily")
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

        combined_hand_actions = [hand_action, hand_sub_action, hand_sub_sub_action]
        return combined_hand_actions


    def tilt_head_back_happily(self):
        rospy.loginfo(rospy.get_caller_id()+" tilting head back happily and nodding")
        self.pub_log.publish(rospy.get_caller_id()+" tilting head back happily and nodding")
        # tilt head back and nod:
        position = 100
        speed = 70
        joint = "head"
        head_action = [[speed, position, joint]]
        sub_speed = speed
        sub_position = position+30
        sub_sub_position = position-30
        # nod head:
        head_sub_action = [[sub_speed, sub_position, joint]]
        head_sub_sub_action = [[sub_speed, sub_sub_position, joint]]

        combined_actions_head = [head_action, head_sub_action, head_sub_sub_action]
        return combined_actions_head


    def hang_hands_down_sadly(self):
        rospy.loginfo(rospy.get_caller_id()+" hanging hands down sadly")
        self.pub_log.publish(rospy.get_caller_id()+" hanging hands down sadly")
        # hang hands down:
        position = 70
        speed = 80
        joint1 = "lhand"
        joint2 = "rhand"
        hand_action = [[speed, position, joint1], [speed, position, joint2]]

        return [hand_action,hand_action,hand_action]
    

    def drop_head_and_shake_sadly(self):
        rospy.loginfo(rospy.get_caller_id()+" dropping head and shaking sadly")
        self.pub_log.publish(rospy.get_caller_id()+" dropping head and shaking sadly")
        # head dropping and shaking:
        position = -20
        speed = 60
        joint1 = "neck"
        joint2 = "head"
        joint2_position = 0
        head_action = [[speed, position, joint1], [speed, joint2_position, joint2]]
        sub_speed = speed
        sub_joint2_position = joint2_position+30
        sub_sub_joint2_position = joint2_position-30

        head_sub_action = [[sub_speed, sub_joint2_position, joint2]]
        head_sub_sub_action = [[sub_speed, sub_sub_joint2_position, joint2]]

        combined_head_actions = [head_action, head_sub_action, head_sub_sub_action]
        return combined_head_actions
        

    def wave_hands_away_fearfully(self):
        rospy.loginfo(rospy.get_caller_id()+" waving hands away fearfully")
        self.pub_log.publish(rospy.get_caller_id()+" waving hands away fearfully")
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

        combined_hand_actions = [hand_action, hand_sub_action, hand_sub_sub_action]
        return combined_hand_actions


    def look_around_rapidly_fearfully(self):
        rospy.loginfo(rospy.get_caller_id()+" looking around rapidly fearfully")
        self.pub_log.publish(rospy.get_caller_id()+" looking around rapidly fearfully")
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

        combined_actions_head = [head_action, head_sub_action, head_sub_sub_action]
        return combined_actions_head
    

    def wave_hands_away_disgustedly(self):
        rospy.loginfo(rospy.get_caller_id()+" waving hands away disgustedly")
        self.pub_log.publish(rospy.get_caller_id()+" waving hands away disgustedly")
        # wave hands away:
        position = 70
        speed = 80
        joint1 = "lhand"
        joint2 = "rhand"
        hand_action = [[speed, position, joint1], [speed, position, joint2]]

        return [hand_action,hand_action,hand_action]
    
    
    def shake_head_disgustedly(self):
        rospy.loginfo(rospy.get_caller_id()+" shaking head disgustedly")
        self.pub_log.publish(rospy.get_caller_id()+" shaking head disgustedly")
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

        combined_actions_head = [head_action, head_sub_action, head_sub_sub_action]
        return combined_actions_head


    
    def wave_hands_away_agressively(self):
        rospy.loginfo(rospy.get_caller_id()+" waving hands away agressively")
        self.pub_log.publish(rospy.get_caller_id()+" waving hands away agressively")
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

        combined_actions_hand = [hand_action, hand_sub_action, hand_sub_sub_action]
        return combined_actions_hand

    def shake_head_aggresively(self):
        rospy.loginfo(rospy.get_caller_id()+" shaking head")
        self.pub_log.publish(rospy.get_caller_id()+" shaking head")
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

        combined_actions_head = [head_action, head_sub_action, head_sub_sub_action]
        return combined_actions_head
    

    def reset_position(self):
        log_message = f'{rospy.get_caller_id()} Reseting all the Dynamixel units to the initial position'
        rospy.loginfo(log_message)
        self.pub_log.publish(log_message)
        # reset the position of the dynamixels
        joint = "reset"
        speed = 100
        position = 10
        reset_action = [[speed, position, joint]]
        self.perform_movement([reset_action])


    def move_to_more_natural_position(self):
        log_message = f"{rospy.get_caller_id()} Moving all the Dynamixel units to more natural positions"
        rospy.loginfo(log_message)
        self.pub_log.publish(log_message)
        # move to more natural positions
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
        position = 0
        neck_action = [[speed, position, joint]]
        
        combined_actions = [lhand_action, rhand_action, head_action, neck_action]
        self.perform_movement(combined_actions)



# function for publishing the movement of hands and head to the topic /cmd_vel/dyna/multiple
    def perform_movement(self, action):
        print(action)
        msg = DynaTwistMultiple()
        for a in action:
            l = DynaTwist()
            l.joint = a[0][2]
            l.speed.angular.x = a[0][0]
            l.position = a[0][1]
            msg.commands.append(l)
        self.pub.publish(msg)
        time.sleep(0.05)
    




if __name__ == '__main__':
    try:
        FeelingsController()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass