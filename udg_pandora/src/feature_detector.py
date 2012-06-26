#!/usr/bin/env python

# ROS imports
import roslib
roslib.load_manifest('udg_pandora')
import rospy
import std_msgs.msg
import std_srvs.srv
from sensor_msgs.msg import PointCloud2
import metaclient

class FeatureDetector:
    def __init__(self, name):
        self.name = name

        # Create Subscriber
        # Subscribe to camera view
        self.visionSub = metaclient.Subscriber("camera", std_msgs.msg.Empty, {}, self.extractFeatures)
        # Subscribe to forward looking sonar
        self.acousticSub = metaclient.Subscriber("forward_look_sonar", std_msgs.msg.Empty, {}, self.extractFeatures)

        # Create publisher
        self.pub_features = metaclient.Publisher('/feature_detector/vision_pcl', PointCloud2,{})
        self.pub_features = metaclient.Publisher('/feature_detector/acoustic_pcl', PointCloud2,{})
        
        #Create services
        self.enable_vision_detection = metaclient.Service('/feature_detector/enable_vision', std_srvs.srv.Empty, self.enableVisionDetectionSrv, {})
        self.disable_vision_detection = metaclient.Service('/feature_detector/disable_vision', std_srvs.srv.Empty, self.disableVisionDetectionSrv, {})
        self.enable_vision_detection = metaclient.Service('/feature_detector/enable_acoustic', std_srvs.srv.Empty, self.enableAcousticDetectionSrv, {})
        self.disable_vision_detection = metaclient.Service('/feature_detector/disable_acoustic', std_srvs.srv.Empty, self.disableAcousticDetectionSrv, {})
        


    def enableVisionDetectionSrv(self, req):
        pass


    def disableVisionDetectionSrv(self, req):
        pass


    def enableAcousticDetectionSrv(self, req):
        pass


    def disableAcousticDetectionSrv(self, req):
        pass


    def extractFeatures(self, img):
        pass


if __name__ == '__main__':
    try:
        rospy.init_node('feature_detector')
        feature_detector = FeatureDetector(rospy.get_name())
        rospy.spin()
    except rospy.ROSInterruptException: pass
