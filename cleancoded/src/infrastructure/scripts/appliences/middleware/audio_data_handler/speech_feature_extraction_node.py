# the pioneer node that subscribes to the audio data topic and send the sudio data to feature extraction server 
#and receives the extracted features in a custom message called AudFeature [which is located in "$(rospack find infrastructure)/msg"]
# then it initiates a topic to publish the received features to the topic /audio_features as a custom message called AudFeature [which is located in "$(rospack find infrastructure)/msg"]

import rospy
from infrastructure.msg import AudFeature
from infrastructure.srv import AudFeature as AudFeatureSrv
from audio_common_msgs.msg import AudioDataStamped as AudioData



rospy.init_node('audio_feature_extractor',anonymous=False)
pub = rospy.Publisher('audio_features', AudFeature, queue_size=10)

def callservice(data):
    rospy.wait_for_service('audio_features')
    try:
        audsrv = rospy.ServiceProxy('audio_features', AudFeatureSrv)
        features = audsrv(data.data)
        rospy.loginfo("audio feature extractor server successfully responded with %s",features)
        rospy.loginfo("Publishing audio features to /audio_features topic")
        pub.publish(features)
        
    except rospy.ServiceException as e:
        rospy.logerr("Service call failed: %s"%e)


if __name__ == '__main__':
    rospy.Subscriber('captured_audio', AudioData, callservice)
    rospy.spin()