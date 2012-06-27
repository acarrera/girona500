#!/usr/bin/env python

# ROS imports
import roslib 
#from mx.DateTime.DateTime import _current_century
roslib.load_manifest('control_g500')
import rospy
import actionlib
from std_srvs.srv import Empty, EmptyResponse, EmptyRequest

# Msgs imports
from auv_msgs.msg import *
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion
from control_g500.msg import *

# Python imports
from numpy import *

# Custom imports
import cola2_lib
from move_mode import * 

class Pilot:
    
    def __init__(self, name):
        self.name = name
        self.rate = 10
        self.nav_sts = NavSts()
        self.current_goal = WorldWaypointReqGoal()
        
        self.getConfig()
        self.move_mode = MoveMode(rospy.Time.now().to_sec(), 
                                  self.pid_x_z_yaw, 
                                  self.pid_x_y_z_yaw, 
                                  self.relative_pid_x_y_z_yaw,
                                  self.max_velocity,
                                  self.min_velocity_los,
                                  self.max_angle_error)
        
        # Create Actionlib
        self.absolute_action = actionlib.SimpleActionServer('absolute_movement', WorldWaypointReqAction, self.actionAbsolute, False)
        self.absolute_action.start()
        # TODO: To be done -->
        # self.relative_action = actionlib.SimpleActionServer('relative_movement', ????, self.actionRelative, False)
        # self.relative_action.start()
                
        # Create publisher
        self.pub_body_velocity_req = rospy.Publisher("/control_g500/body_velocity_req", BodyVelocityReq)
    
        # Create Subscriber
        rospy.Subscriber("/navigation_g500/nav_sts", NavSts, self.updateNavSts)
        
        #Initialize previous req (necessary for LOS)
        self.previous_goal = WorldWaypointReqGoal()
        self.previous_goal.goal.requester = self.name

        
    def getConfig(self) :
        """ Load parameters from the rosparam server """
        
        if rospy.has_param("pilot/pid_x_z_yaw_feed_forward_force"):
            xzyaw_fff = array(rospy.get_param("pilot/pid_x_z_yaw_feed_forward_force"))
        else:
            rospy.logfatal("pilot/pid_x_z_yaw_feed_forward_force param not found")
        
        if rospy.has_param("pilot/pid_x_z_yaw_kp") :
            xzyaw_kp = array(rospy.get_param("pilot/pid_x_z_yaw_kp"))
        else:
            rospy.logfatal("pilot/pid_x_z_yaw_kp param not found")

        if rospy.has_param("pilot/pid_x_z_yaw_ti") :
            xzyaw_ti = array(rospy.get_param("pilot/pid_x_z_yaw_ti"))
        else:
            rospy.logfatal("pilot/pid_x_z_yaw_ti param not found")

        if rospy.has_param("pilot/pid_x_z_yaw_td") :
            xzyaw_td = array(rospy.get_param("pilot/pid_x_z_yaw_td"))
        else:
            rospy.logfatal("pilot/pid_x_z_yaw_td param not found")

        self.pid_x_z_yaw = cola2_lib.PID(xzyaw_kp, xzyaw_ti, xzyaw_td, xzyaw_fff, rospy.Time.now().to_sec())
        
        if rospy.has_param("pilot/pid_x_y_z_yaw_feed_forward_force"):
            xyzyaw_fff = array(rospy.get_param("pilot/pid_x_y_z_yaw_feed_forward_force"))
        else:
            rospy.logfatal("pilot/pid_x_y_z_yaw_feed_forward_force param not found")
        
        if rospy.has_param("pilot/pid_x_y_z_yaw_kp") :
            xyzyaw_kp = array(rospy.get_param("pilot/pid_x_y_z_yaw_kp"))
        else:
            rospy.logfatal("pilot/pid_x_y_z_yaw_kp param not found")

        if rospy.has_param("pilot/pid_x_y_z_yaw_ti") :
            xyzyaw_ti = array(rospy.get_param("pilot/pid_x_y_z_yaw_ti"))
        else:
            rospy.logfatal("pilot/pid_x_y_z_yaw_ti param not found")

        if rospy.has_param("pilot/pid_x_y_z_yaw_td") :
            xyzyaw_td = array(rospy.get_param("pilot/pid_x_y_z_yaw_td"))
        else:
            rospy.logfatal("pilot/pid_x_y_z_yaw_td param not found")

        self.pid_x_y_z_yaw = cola2_lib.PID(xyzyaw_kp, xyzyaw_ti, xyzyaw_td, xyzyaw_fff, rospy.Time.now().to_sec())
        
        if rospy.has_param("pilot/relative_pid_x_y_z_yaw_feed_forward_force"):
            relative_xyzyaw_fff = array(rospy.get_param("pilot/relative_pid_x_y_z_yaw_feed_forward_force"))
        else:
            rospy.logfatal("pilot/relative_pid_x_y_z_yaw_feed_forward_force param not found")
        
        if rospy.has_param("pilot/relative_pid_x_y_z_yaw_kp") :
            relative_xyzyaw_kp = array(rospy.get_param("pilot/relative_pid_x_y_z_yaw_kp"))
        else:
            rospy.logfatal("pilot/relative_pid_x_y_z_yaw_kp param not found")

        if rospy.has_param("pilot/relative_pid_x_y_z_yaw_ti") :
            relative_xyzyaw_ti = array(rospy.get_param("pilot/relative_pid_x_y_z_yaw_ti"))
        else:
            rospy.logfatal("pilot/relative_pid_x_y_z_yaw_ti param not found")

        if rospy.has_param("pilot/relative_pid_x_y_z_yaw_td") :
            relative_xyzyaw_td = array(rospy.get_param("pilot/relative_pid_x_y_z_yaw_td"))
        else:
            rospy.logfatal("pilot/relative_pid_x_y_z_yaw_td param not found")

        self.relative_pid_x_y_z_yaw = cola2_lib.PID(relative_xyzyaw_kp, relative_xyzyaw_ti, 
                                                    relative_xyzyaw_td, relative_xyzyaw_fff, 
                                                    rospy.Time.now().to_sec())
        
        if rospy.has_param("pilot/max_velocity") :
            self.max_velocity = array(rospy.get_param("pilot/max_velocity"))
        else:
            rospy.logfatal("pilot/max_velocity param not found")

        if rospy.has_param("pilot/min_velocity_los") :
            self.min_velocity_los = array(rospy.get_param("pilot/min_velocity_los"))
        else:
            rospy.logfatal("pilot/min_velocity_los param not found")

        if rospy.has_param("pilot/max_angle_error") :
            self.max_angle_error = array(rospy.get_param("pilot/max_angle_error"))
        else:
            rospy.logfatal("pilot/max_angle_error param not found")
     

    def actionAbsolute(self, goal):
        success = False
        preempted = False
        self.current_goal = goal;
        
        rospy.loginfo('RECEIVED ABSOLUTE ACTION: \n%s', goal)
         
        #if is the first waypoint to achieve, initialize the previous one with current position
        if goal.goal.id == 0:
            self.previous_goal.position.north = self.nav_sts.position.north
            self.previous_goal.position.east = self.nav_sts.position.east

        if self.move_mode.legacyReq(goal, rospy.Time.now().to_sec()):
            mode = self.move_mode.checkAbsoluteNavigationMode(goal.disable_axis)
            if mode == MovementMode.ABSOLUTE_X_Z_YAW:
                rospy.loginfo('%s, MovementMode.ABSOLUTE_X_Z_YAW', self.name)
                r = rospy.Rate(self.rate)
                while not success and not preempted:
                    #Call Move X Z YAW Method. There are two available modes:
                    # moveMode_X_Z_YAW (default) or moveMode_LOS
                    if goal.mode == 'los':
                        [success, bvr] = self.move_mode.moveMode_LOS(self.previous_goal, 
                                                                     goal, 
                                                                     rospy.Time.now().to_sec())
                    else:
                        [success, bvr] = self.move_mode.moveMode_X_Z_YAW(goal, 
                                                                         rospy.Time.now().to_sec())
                    
                    # Publish body_velocity_req
                    self.pub(bvr, goal)
                    
                    #If preempted
                    if self.absolute_action.is_preempt_requested():
                        rospy.loginfo('%s: Preempted ABSOLUTE_X_Z_YAW', self.name)
                        self.absolute_action.set_preempted()
                        preempted = True
                    else :
                        #Create Feedback response
                        self.pubAbsoluteFeedback(goal)
                        #Sleep
                        r.sleep()
                        
                if success:
                    #save the last waypoint
                    self.previous_goal = goal
                    self.pubAbsoluteResult(goal)
                    
            elif mode == MovementMode.ABSOLUTE_X_Y_Z_YAW:
                rospy.loginfo('%s, MovementMode.ABSOLUTE_X_Y_Z_YAW', self.name)
                r = rospy.Rate(self.rate)
                while not success and not preempted:
                    #Call Move X Z YAW Method
                    [success, bvr] = self.move_mode.moveMode_X_Y_Z_YAW(goal, 
                                                                       rospy.Time.now().to_sec())
                    if goal.mode == 'neverending':
                        success = False
                    
                    #Publish body_velocity_req
                    self.pub(bvr, goal)
                    
                    #If preempted
                    if self.absolute_action.is_preempt_requested():
                        rospy.loginfo('%s: Preempted ABSOLUTE_X_Y_Z_YAW', self.name)
                        self.absolute_action.set_preempted()
                        preempted = True
                    else :
                        #Create Feedback response
                        self.pubAbsoluteFeedback(goal)
                        #Sleep
                        r.sleep()
                        
                if success:
                    #save the last waypoint
                    self.previous_goal = goal
                    self.pubAbsoluteResult(goal)
            else:
                rospy.logwarn("Invalid movement mode")
        else:
            rospy.logwarn("Ilegal world waypoint request")        


    def pub(self, v, req):
        v = around(v, 2) #Trunca a 2 decimals
        body_velocity_req =  BodyVelocityReq()
        body_velocity_req.header.stamp = rospy.Time.now()
        body_velocity_req.goal = req.goal
        body_velocity_req.twist.linear.x = v[0]
        body_velocity_req.twist.linear.y = v[1]
        body_velocity_req.twist.linear.z = v[2]
        body_velocity_req.twist.angular.x = v[3]
        body_velocity_req.twist.angular.y = v[4]
        body_velocity_req.twist.angular.z = v[5]
        body_velocity_req.disable_axis = req.disable_axis
        
        rospy.loginfo("Send Body Velocity Req:\n%s", str(v)) 
        self.pub_body_velocity_req.publish(body_velocity_req)
        
        
    def updateNavSts(self, nav_sts):
        self.nav_sts = nav_sts
        p = zeros(6)
        v = zeros(3)
        p[0] = nav_sts.position.north
        p[1] = nav_sts.position.east
        p[2] = nav_sts.position.depth
        p[3] = nav_sts.orientation.roll
        p[4] = nav_sts.orientation.pitch
        p[5] = nav_sts.orientation.yaw
        v[0] = nav_sts.body_velocity.x
        v[1] = nav_sts.body_velocity.y
        v[2] = nav_sts.body_velocity.z
        
        self.move_mode.updateNav(p, v, nav_sts.altitude)
        
        
    def pubAbsoluteFeedback(self, goal):
        feedback = WorldWaypointReqFeedback()
        feedback.altitude_mode = goal.altitude_mode
        feedback.position.north = self.nav_sts.position.north
        feedback.position.east = self.nav_sts.position.east
        feedback.position.depth = self.nav_sts.position.depth
        feedback.altitude = self.nav_sts.altitude
        feedback.orientation.roll = self.nav_sts.orientation.roll
        feedback.orientation.pitch = self.nav_sts.orientation.pitch
        feedback.orientation.yaw = self.nav_sts.orientation.yaw
        self.absolute_action.publish_feedback(feedback)
        # rospy.loginfo("Feedback:\n%s", feedback)

    
    def pubAbsoluteResult(self, goal):
        result = WorldWaypointReqResult()
        result.altitude_mode = goal.altitude_mode
        result.position.north = self.nav_sts.position.north
        result.position.east = self.nav_sts.position.east
        result.position.depth = self.nav_sts.position.depth
        result.altitude = self.nav_sts.altitude
        result.orientation.roll = self.nav_sts.orientation.roll
        result.orientation.pitch = self.nav_sts.orientation.pitch
        result.orientation.yaw = self.nav_sts.orientation.yaw
        rospy.loginfo('%s: Succeeded', self.name)
        self.absolute_action.set_succeeded(result)


if __name__ == '__main__':
    try:
        rospy.init_node('pilot')
        pilot = Pilot(rospy.get_name())
        rospy.spin() 
    except rospy.ROSInterruptException: pass
