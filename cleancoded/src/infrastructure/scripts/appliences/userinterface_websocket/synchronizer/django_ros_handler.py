# import roslaunch

# package = 'rqt_gui'
# executable = 'rqt_gui'
# node = roslaunch.core.Node(package, executable)

# launch = roslaunch.scriptapi.ROSLaunch()
# launch.start()

# process = launch.launch(node)
# print process.is_alive()
# process.stop()


# import roslaunch

# roslaunch.configure_logging(uuid)
# launch = roslaunch.scriptapi.ROSLaunch()
# launch.parent = roslaunch.parent.ROSLaunchParent(uuid, "path/to/base.launch")
# launch.start()

# # Start another node
# node = roslaunch.core.Node(package, executable)
# launch.launch(node)

# try:
#   launch.spin()
# finally:
#   # After Ctrl+C, stop all nodes from running
#   launch.shutdown()


import rospy
from std_msgs.msg import String
from infrastructure.msg import *

class ROSHandler:
    rospy.init_node('ROS_Django_Node',anonymous=False)

    def __init__(self):
        pass
        
        
        
    def topics_list(self):
        return rospy.get_published_topics(namespace='/') #Returns: [[str, str]] 
                                                        # List of topic names and types: [[topic1, type1]...[topicN, typeN]]
                                                        # Source: https://docs.ros.org/en/kinetic/api/rospy/html/rospy.client-module.html#get_published_topics
        

    def get_image_topics(self):
        topics = self.topics_list()
        image_topics = []
        for topic, msg_type in topics:
            if msg_type == 'sensor_msgs/Image':
                image_topics.append(topic)
        return image_topics
    

    def topic_type(self, topic):
        req_topic_list = [topic_list for topic_list in self.topics_list() if topic_list[0] == topic]
        if len(req_topic_list) != 0:
            return req_topic_list[0][1] # Returns the type of the topic as a string
        else:
            return 'Topic not found'
        

    def msg_coupling(self, topic):
        msg_type = self.topic_type(topic)

        string_msg = ['data']
        twist_msg = {'linear': ['x', 'y', 'z'], 'angular': ['x', 'y', 'z']}
        dyan_twist_msg = [twist_msg,'position','joint']
        exp_msg = ['action','emotion','auto_imit']
        


        if msg_type == 'std_msgs/String':
            return string_msg
        