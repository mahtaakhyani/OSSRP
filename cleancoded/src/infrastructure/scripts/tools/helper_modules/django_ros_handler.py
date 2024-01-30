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
import importlib
import roslib
import rosmsg

class ROSHandler:
    rospy.init_node('ROS_Django_Node',anonymous=False)

    def __init__(self):
        self.topics = rospy.get_published_topics(namespace='/') #Returns: [[str, str]] 
                                                        # List of topic names and types: [[topic1, type1]...[topicN, typeN]]
                                                        # Source: https://docs.ros.org/en/kinetic/api/rospy/html/rospy.client-module.html#get_published_topics
        self.topics_name = [topic[0] for topic in self.topics]

    
    def get_all_topics(self):
        return self.topics_name

    def get_image_topics(self):
        image_topics = []
        for topic, msg_type in self.topics:
            if msg_type == 'sensor_msgs/Image':
                image_topics.append(topic)
        return image_topics
    

    def get_topic_type(self, topic):
        req_topic_list = [topic_list for topic_list in self.topics if topic_list[0] == topic]
        if len(req_topic_list) != 0:
            msg_type = req_topic_list[0][1]
            msg_class = roslib.message.get_message_class(msg_type)
            msg_obj = msg_class()
            # pkg_name = f'{msg_type.split("/")[0]}.msg'
            # pkg = importlib.import_module(pkg_name)
            # msg_name = msg_type.split("/")[1]
            # msg = getattr(pkg, msg_name)
            msg_sub_fields = msg_obj.__slots__
            print(msg_sub_fields)
            for field in msg_sub_fields:
            #     # try:
            #     print(field)
                sub_field = getattr(msg_obj, str(field))
                print(sub_field)
            #     # msg_sub_fields.pop(field)
            #     # msg_sub_fields.append([sub_field])
            #     # except:
            #     #     pass
            return msg_sub_fields # Returns the type of the topic (i.e. msg type) as a string
        else:
            return 'Topic not found'
        
