#! /usr/bin/env python

#  logitech f510 joystick driver
#  Created on: 17/06/2011
#  Updated on: 11/01/2012
#  Author: narcis

# ROS imports
import roslib 
roslib.load_manifest('control_g500')
import rospy

# import messages
from sensor_msgs.msg import Joy
from std_msgs.msg import *
from safety_g500.srv import DigitalOutput, DigitalOutputResponse, DigitalOutputRequest


class LogitechF510Joystick:
    def __init__(self, name):
		self.name = name
		self.getConfig()
		
		# Create publisher
		self.pub_joy_data = rospy.Publisher("/control_g500/joystick_data", Joy)
		self.pub_joy_ack_teleop = rospy.Publisher("/control_g500/joystick_ack", String)
		
		# Create Subscriber
		rospy.Subscriber("joy", Joy, self.updateJoy)
		rospy.Subscriber("/control_g500/joystick_ok", String, self.updateAck)
		
		#Create Timer
		rospy.Timer(rospy.Duration(0.1), self.pubJoyData)
		
		#Create Joy msg and init 4 axes (X, Y, Z, Roll, Pitch & Yaw)
		self.joy_data = Joy()
		for i in range(6): 
			self.joy_data.axes.append(0.0)

		rospy.loginfo("%s, initialized", self.name)
        
        
    def getConfig(self):
        if rospy.has_param("joy/min_x_v"):
            self.min_x_v = rospy.get_param('joy/min_x_v')
        else:
            rospy.logfatal("joy/min_x_v param not found")
       	
        if rospy.has_param("joy/max_x_v") :
            self.max_x_v = rospy.get_param('joy/max_x_v')
        else:
            rospy.logfatal("joy/max_x_v param not found")
        
        if rospy.has_param("joy/min_y_v") :
            self.min_y_v = rospy.get_param('joy/min_y_v')
        else:
            rospy.logfatal("joy/min_y_v param not found")
           
        if rospy.has_param("joy/max_y_v") :
            self.max_y_v = rospy.get_param('joy/max_y_v')
        else:
            rospy.logfatal("joy/max_y_v param not found")
        
        if rospy.has_param("joy/min_z_v") :
            self.min_z_v = rospy.get_param('joy/min_z_v')
        else:
            rospy.logfatal("joy/min_z_v param not found")
        
        if rospy.has_param("joy/max_z_v") :
            self.max_z_v = rospy.get_param('joy/max_z_v')
        else:
            rospy.logfatal("joy/max_z_v param not found")
        
        if rospy.has_param("joy/min_yaw_v") :
            self.min_yaw_v = rospy.get_param('joy/min_yaw_v')
        else:
            rospy.logfatal("joy/min_yaw_v param not found")
        
        if rospy.has_param("joy/max_yaw_v") :
            self.max_yaw_v = rospy.get_param('joy/max_yaw_v')
        else:
            rospy.logfatal("joy/max_yaw_v param not found")
           
 
    def pubJoyData(self, event):
 		self.pub_joy_data.publish(self.joy_data)
 		
 		
    def updateJoy(self, data):
        if len(data.axes) > 4:
	    self.joy_data.axes[0] = data.axes[1]
	    self.joy_data.axes[5] = -data.axes[3]
	    self.joy_data.axes[2] = -data.axes[4]
	    self.joy_data.axes[1] = -data.axes[0]
		
            if self.joy_data.axes[0] > 0.0: self.joy_data.axes[0] *= self.max_x_v
            else: self.joy_data.axes[0] *= -self.min_x_v
            if self.joy_data.axes[1] > 0.0: self.joy_data.axes[1] *= self.max_y_v
            else: self.joy_data.axes[1] *= -self.min_y_v
            if self.joy_data.axes[2] > 0.0: self.joy_data.axes[2] *= self.max_z_v
            else: self.joy_data.axes[2] *= -self.min_z_v
            if self.joy_data.axes[5] > 0.0: self.joy_data.axes[5] *= self.max_yaw_v
            else: self.joy_data.axes[5] *= -self.min_yaw_v
        else:
            rospy.logerr("%s, Invalid /joy message received!", self.name)

        if len(data.buttons) > 2:
            if data.buttons[1] == 1 and data.buttons[2] == 1: 
                rospy.loginfo("%s, STOP payload", self.name)
                try:
                    rospy.wait_for_service('digital_output', 5)
                    digital_out_service = rospy.ServiceProxy('digital_output', DigitalOutput)
                    digital_output = DigitalOutputRequest()
                    digital_output.digital_out = 5
                    digital_output.value = False
                    digital_out_service(digital_output)
                except rospy.exceptions.ROSException:
                    rospy.logerr('%s, Error disabling payload.', self.name)
         

    def updateAck(self, ack):
 		ack_list = ack.data.split()
 		if len(ack_list) == 2 and ack_list[1] == 'ok':
 			seq = int(ack_list[0]) + 1
 			self.pub_joy_ack_teleop.publish(str(seq) + " ack")
 		else:
 			rospy.logerr("%s, received invalid teleoperation heart beat!", self.name)
 	

if __name__ == '__main__':
    try:
        rospy.init_node('logitech_f510_joystick')
        joystick = LogitechF510Joystick(rospy.get_name())
        rospy.spin() 
    except rospy.ROSInterruptException: pass
