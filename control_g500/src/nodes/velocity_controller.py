#!/usr/bin/env python

# ROS imports
import roslib 
roslib.load_manifest('control_g500')
import rospy

# Msgs imports
from auv_msgs.msg import *
from nav_msgs.msg import Odometry
from std_srvs.srv import Empty, EmptyResponse, EmptyRequest
from std_msgs.msg import Float32

# Python imports
from numpy import *

# Custom imports
import cola2_lib


class VelocityController :
    def __init__(self, name):
        """ Controls the velocity and pose of an AUV  """
        self.name = name
        
        # Load parameters
        self.getConfig()
        self.enable = False
        
        # Input data 
        self.v = zeros(6)
        self.desired_velocity = zeros(6)
        self.resp = BodyVelocityReq()
        
        # Create publisher
        self.pub_tau = rospy.Publisher("/control_g500/velocity_to_force_req", BodyForceReq)
        
        # Create Subscriber
        # rospy.Subscriber("/navigation_g500/odometry", Odometry, self.updateOdometry)
        rospy.Subscriber("/navigation_g500/nav_sts", NavSts, self.updateNavSts)
        rospy.Subscriber("/control_g500/merged_body_velocity_req", BodyVelocityReq, self.updateResponse)
        
        #Create services
        self.enable_srv = rospy.Service('/control_g500/enable_velocity_controller', Empty, self.enableSrv)
        self.disable_srv = rospy.Service('/control_g500/disable_velocity_controller', Empty, self.disableSrv)


    def getConfig(self) :
        """ Load parameters from the rosparam server """
        #TODO: Check that the parameters are in the param server
        if rospy.has_param("velocity_controller/force_max") :
            self.force_max = array( rospy.get_param("velocity_controller/force_max"))
        else:
            rospy.logfatal("velocity_controller/force_max param not found")

        if rospy.has_param("velocity_controller/pid_velocity_feed_forward_force") :
            fff = array( rospy.get_param("velocity_controller/pid_velocity_feed_forward_force") )
        else:
            rospy.logfatal("velocity_controller/pid_velocity_feed_forward_force param not found")
        
        if rospy.has_param("velocity_controller/pid_velocity_kp") :
            kp = array(rospy.get_param("velocity_controller/pid_velocity_kp"))
        else:
            rospy.logfatal("velocity_controller/pid_velocity_kp param not found")

        if rospy.has_param("velocity_controller/pid_velocity_ti") :
            ti = array(rospy.get_param("velocity_controller/pid_velocity_ti"))
        else:
            rospy.logfatal("velocity_controller/pid_velocity_ti param not found")

        if rospy.has_param("velocity_controller/pid_velocity_td") :
            td = array(rospy.get_param("velocity_controller/pid_velocity_td"))
        else:
            rospy.logfatal("velocity_controller/pid_velocity_td param not found")
                 
        self.adjust_poly = []

        if rospy.has_param("velocity_controller/open_loop_adjust_poly_x") :
            self.adjust_poly.append(rospy.get_param("velocity_controller/open_loop_adjust_poly_x"))
        else:
            rospy.logfatal("velocity_controller/open_loop_adjust_poly_x")
        
        if rospy.has_param("velocity_controller/open_loop_adjust_poly_y") :
            self.adjust_poly.append(rospy.get_param("velocity_controller/open_loop_adjust_poly_y"))
        else:
            rospy.logfatal("velocity_controller/open_loop_adjust_poly_y")
        
        if rospy.has_param("velocity_controller/open_loop_adjust_poly_z") :
            self.adjust_poly.append(rospy.get_param("velocity_controller/open_loop_adjust_poly_z"))
        else:
            rospy.logfatal("velocity_controller/open_loop_adjust_poly_z")
    
        if rospy.has_param("velocity_controller/open_loop_adjust_poly_roll") :
            self.adjust_poly.append(rospy.get_param("velocity_controller/open_loop_adjust_poly_roll"))
        else:
            rospy.logfatal("velocity_controller/open_loop_adjust_poly_roll")
            
        if rospy.has_param("velocity_controller/open_loop_adjust_poly_pitch") :
            self.adjust_poly.append(rospy.get_param("velocity_controller/open_loop_adjust_poly_pitch"))
        else:
            rospy.logfatal("velocity_controller/open_loop_adjust_poly_pitch")
            
        if rospy.has_param("velocity_controller/open_loop_adjust_poly_yaw") :
            self.adjust_poly.append(rospy.get_param("velocity_controller/open_loop_adjust_poly_yaw"))
        else:
            rospy.logfatal("velocity_controller/open_loop_adjust_poly_yaw")
            
        rospy.loginfo('%s, Adjust Poly:\n %s', self.name, str(self.adjust_poly))
        rospy.loginfo('%s, Kp: %s', self.name, str(kp))
        rospy.loginfo('%s, Ti: %s', self.name, str(ti))
        rospy.loginfo('%s, Td: %s', self.name, str(td))
        rospy.loginfo('%s, fff: %s', self.name, str(fff))
        
        self.pid = cola2_lib.PID(kp, ti, td, fff, rospy.Time.now().to_sec())
    
    
    def enableSrv(self, req):
        self.enable = True
        rospy.loginfo('%s Enabled', self.name)
        return EmptyResponse()
    
        
    def disableSrv(self, req):
        self.enable = False
        rospy.loginfo('%s Disabled', self.name)
        return EmptyResponse()
    
    
#    def updateOdometry(self, odom):
#        self.v[0] = odom.twist.twist.linear.x
#        self.v[1] = odom.twist.twist.linear.y
#        self.v[2] = odom.twist.twist.linear.z
#        self.v[3] = odom.twist.twist.angular.x
#        self.v[4] = odom.twist.twist.angular.y
#        self.v[5] = odom.twist.twist.angular.z
    
    
    def updateNavSts(self, nav_sts):
        self.v[0] = nav_sts.body_velocity.x
        self.v[1] = nav_sts.body_velocity.y
        self.v[2] = nav_sts.body_velocity.z
        self.v[3] = nav_sts.orientation_rate.roll
        self.v[4] = nav_sts.orientation_rate.pitch
        self.v[5] = nav_sts.orientation_rate.yaw
        
    
    def updateResponse(self, resp):
        self.desired_velocity[0] = resp.twist.linear.x
        self.desired_velocity[1] = resp.twist.linear.y
        self.desired_velocity[2] = resp.twist.linear.z
        self.desired_velocity[3] = resp.twist.angular.x
        self.desired_velocity[4] = resp.twist.angular.y
        self.desired_velocity[5] = resp.twist.angular.z
        self.resp = resp
        self.iterate()
         
         
    def iterate(self): 
        if self.enable:
            # Main loop
            rospy.loginfo("desired_velocity: %s", str(self.desired_velocity))
            rospy.loginfo("current velocity: %s", str(self.v))
            
            # Compute TAU using a PID
            pid_tau  = self.pid.compute(self.desired_velocity, self.v, rospy.Time.now().to_sec())
            
            # Compute TAU using an Open Loop Controller
            open_loop_tau = zeros(6)
            for i in range(6):
                open_loop_tau[i] = cola2_lib.polyval(self.adjust_poly[i], self.desired_velocity[i])
            
            # Combine open loop TAU and PID TAU
            tau = cola2_lib.saturateValue(pid_tau + open_loop_tau, 1.0) * self.force_max
            
            rospy.loginfo("Tau: %s", str(tau))
            
            data = BodyForceReq()
            data.header.stamp = rospy.Time.now()
            data.header.frame_id = "vehicle_frame"
            data.goal.requester = "velocity_controller"
            data.goal.id = 0
            data.goal.priority = GoalDescriptor.PRIORITY_NORMAL
            data.wrench.force.x = tau[0]
            data.wrench.force.y = tau[1]
            data.wrench.force.z = tau[2]        
            data.wrench.torque.x = tau[3]
            data.wrench.torque.y = tau[4]
            data.wrench.torque.z = tau[5]
            
            data.disable_axis.x = self.resp.disable_axis.x
            data.disable_axis.y = self.resp.disable_axis.y
            data.disable_axis.z = self.resp.disable_axis.z
            data.disable_axis.roll = self.resp.disable_axis.roll
            data.disable_axis.pitch = self.resp.disable_axis.pitch
            data.disable_axis.yaw = self.resp.disable_axis.yaw
            
            self.pub_tau.publish(data)


if __name__ == '__main__':
    try:
        rospy.init_node('velocity_controller')
        velocity_controller = VelocityController(rospy.get_name())
        rospy.spin()

    except rospy.ROSInterruptException: pass
