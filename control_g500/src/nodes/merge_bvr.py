#!/usr/bin/env python

# ROS imports
import roslib 
roslib.load_manifest('control_g500')
import rospy

# Msgs imports
from auv_msgs.msg import *

# Python imports
from numpy import *

class MergeBvr :
    
    def updateResponse(self, resp):
        #Check if the requester has any old message in sp. If there is one, remove it.
        for m in self.sp:
            if m.goal.requester == resp.goal.requester:
                self.sp.remove(m)
                #print "Remove requester", resp.goal.requester
        self.sp.append(resp)
        #rospy.loginfo("Received response from: %s", resp.goal.requester)
    
    
    def sortResponses(self, sp):
        ret = []
        
        #while elements in the sp vector
        while len(sp) > 0 :
            max_priority = 0
            index = -1
            #Search the response with more priority
            for i in range(len(sp)) :
                if sp[i].goal.priority > max_priority :
                    max_priority = sp[i].goal.priority
                    index = i
            #Add the most priority response to ret and remove it from the sp vector
            ret.append(sp[index])
            sp.remove(sp[index])
        
        return ret
    
    
    def match(self, req_axis, setted_axis):
        req_axis_v = [req_axis.x, req_axis.y, req_axis.z, req_axis.roll, req_axis.pitch, req_axis.yaw]
        setted_axis_v = [setted_axis.x, setted_axis.y, setted_axis.z, setted_axis.roll, setted_axis.pitch, setted_axis.yaw]
        match = True
        i = 0    
        while match and i < 6:
            if not(req_axis_v[i]) :
                if not(setted_axis_v[i]) :
                    match = False
            i += 1
    
        return match
    
        
    def merge(self, sp):
        ret = BodyVelocityReq()
        ret.disable_axis.x = True
        ret.disable_axis.y = True
        ret.disable_axis.z = True
        ret.disable_axis.roll = True
        ret.disable_axis.pitch = True
        ret.disable_axis.yaw = True
        ret.twist.linear.x = 0.0
        ret.twist.linear.y = 0.0
        ret.twist.linear.z = 0.0
        ret.twist.angular.x = 0.0
        ret.twist.angular.y = 0.0
        ret.twist.angular.z = 0.0
        ret.goal.priority = GoalDescriptor.PRIORITY_LOW
        ret.goal.requester = self.name
        
        print "self.sp: ", len(self.sp)
        if len(sp) == 0 :
            ret.disable_axis.x = False
            ret.disable_axis.y = False
            ret.disable_axis.z = False
            ret.disable_axis.yaw = False
            rospy.loginfo("No merged responses")
            return ret
        elif len(sp) == 1 :
            #print "coordinator: there is only one response"
            rospy.loginfo("One merged response")
            return sp[0]
        else :
            sp = self.sortResponses(sp)
            ret.goal.priority = sp[0].goal.priority
            for resp in sp :
                if self.match(resp.disable_axis, ret.disable_axis) :
                    if not(resp.disable_axis.x) : 
                        ret.disable_axis.x = False
                        ret.twist.linear.x = resp.twist.linear.x
                    if not(resp.disable_axis.y) : 
                        ret.disable_axis.y = False
                        ret.twist.linear.y = resp.twist.linear.y
                    if not(resp.disable_axis.z) : 
                        ret.disable_axis.z = False
                        ret.twist.linear.z = resp.twist.linear.z
                    if not(resp.disable_axis.roll) : 
                        ret.disable_axis.roll = False
                        ret.twist.angular.x = resp.twist.angular.x
                    if not(resp.disable_axis.pitch) : 
                        ret.disable_axis.pitch = False
                        ret.twist.angular.y = resp.twist.angular.y
                    if not(resp.disable_axis.yaw) : 
                        ret.disable_axis.yaw = False
                        ret.twist.angular.z = resp.twist.angular.z
            rospy.loginfo("%d merged responses", len(sp))
            return ret
        
        
    def __init__(self, name):
        """ Controls the velocity and pose of an AUV  """
        self.name = name
    # Input data 
        self.sp = []
    
    #   Create publisher
        self.pub_coordinated = rospy.Publisher("/control_g500/merged_body_velocity_req", BodyVelocityReq)
    
    #   Create Subscriber
        rospy.Subscriber("/control_g500/body_velocity_req", BodyVelocityReq, self.updateResponse)
        

         
    def iterate(self):
        merged_response = self.merge(self.sp)
        rospy.loginfo("merge_bvr response: %s, %s, %s %s. Priority: %s", merged_response.twist.linear.x, merged_response.twist.linear.y, merged_response.twist.linear.z, merged_response.twist.angular.z, merged_response.goal.priority)
        self.pub_coordinated.publish(merged_response)
        #instead of remove all the messages remove those whose time stamps has expired (> 2 second)
        now = rospy.Time.now()
        for m in self.sp:
            if (now - m.header.stamp).to_sec() > 2.0:
                self.sp.remove(m)
                rospy.logerr(" !!!!!!!!!!!  Remove %s", m.goal.requester)
        #self.sp=[]
        
            

if __name__ == '__main__':
    try:
        rospy.init_node('merge_bvr')
        merge_bvr = MergeBvr(rospy.get_name())
        r = rospy.Rate(10)
        while not rospy.is_shutdown():
            merge_bvr.iterate()
            r.sleep()
    except rospy.ROSInterruptException: pass

