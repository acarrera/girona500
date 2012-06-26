# -*- coding: utf-8 -*-
"""
Created on Mon Jun 25 14:10:01 2012

@author: snagappa
"""

# ROS imports
import roslib
roslib.load_manifest('udg_pandora')
import rospy
from auv_msgs.msg import NavSts
from sensor_msgs.msg import PointCloud2
import metaclient


class PhdSlam():
    def __init__(self, name):
        self.name = name
        
        # Subscribe to visual features node
        self.visionSub = metaclient.Subscriber("/feature_detector/vision_pcl", PointCloud2, self.update_features)
        # Subscribe to acoustic features node
        self.acousticSub = metaclient.Subscriber("/feature_detector/acoustic_pcl", PointCloud2, self.update_features)
        
        # Create publisher
        self.pub_nav_sts = metaclient.Publisher("/phdslam/nav_sts", NavSts, {})
        self.pub_map = metaclient.Publisher("/phdslam/map", PointCloud2, {})
        
    def update_features(self):
        pass
    

if __name__ == '__main__':
    try:
        rospy.init_node('phdslam')
        phdslam = PhdSlam(rospy.get_name())
        rospy.spin()
    except rospy.ROSInterruptException: pass
