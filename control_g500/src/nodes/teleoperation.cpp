/*
 * teleoperation_vel.cpp
 *
 *  Created on: 22/05/2011
 *  updated on: 11/01/2012
 *      Author: arnau & narcis
 */

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sensor_msgs/Joy.h"
#include "auv_msgs/BodyVelocityReq.h"
#include "auv_msgs/GoalDescriptor.h"


class Teleoperation{

private:
	ros::NodeHandle _n;
	ros::Publisher _pub_body_velocity_req;
	ros::Publisher _pub_check_joystick;
	ros::Subscriber _sub_ack;
	ros::Subscriber _sub_data;
	ros::Timer _t;
	unsigned int _seq;
	bool _joystick_alive; 
	bool _joystick_init;


public:
	Teleoperation() { }
	~Teleoperation() { }

	void init()
	{
		_seq = 0;
	
		//Publishers
		_pub_body_velocity_req = _n.advertise<auv_msgs::BodyVelocityReq>("/control_g500/body_velocity_req", 1);
		_pub_check_joystick = _n.advertise<std_msgs::String >("/control_g500/joystick_ok", 1);

		//Subscribers
		_sub_ack = _n.subscribe("/control_g500/joystick_ack", 1, &Teleoperation::joystickAckCallback, this);
		_sub_data = _n.subscribe("/control_g500/joystick_data", 1, &Teleoperation::joystickDataCallback, this);

		_joystick_init = false;
		_joystick_alive = true ;
		_t = _n.createTimer(ros::Duration(1.0), &Teleoperation::checkJoystick, this) ; // 1Hz
	}



	/**
	 * This method is a callback of a JoystickData message published.
	 * Every time it's call it converts the JoystickData into a Twist Message
	 *
	 * @param event Is not used. ROS must use it in this kind of callbacks
	 *
	 * @author Arnau Carrera
	 * @date 22/05/2011
	 */
	void joystickDataCallback(const ros::MessageEvent<sensor_msgs::Joy const>& event)
	{
		const std::string& publisher_name = event.getPublisherName();
		const sensor_msgs::Joy::ConstPtr& msg = event.getMessage();

		ROS_INFO("JoystickData received from: [%s]", publisher_name.c_str());

		////////////////////////////////////////////////
		//Define _body_velocity_req message
		////////////////////////////////////////////////
		auv_msgs::BodyVelocityReq body_velocity_req;
		
		//header
		body_velocity_req.header.stamp = ros::Time::now();

		//goal
		body_velocity_req.goal.priority = auv_msgs::GoalDescriptor::PRIORITY_MANUAL_OVERRIDE;
		body_velocity_req.goal.requester = "teleoperation";
				
		//twist set-point
		body_velocity_req.twist.linear.x = msg->axes[0];
		body_velocity_req.twist.linear.y = msg->axes[1];
		body_velocity_req.twist.linear.z = msg->axes[2];
		body_velocity_req.twist.angular.z = msg->axes[5];
		
		//disabled_axis boby_velocity_req
		body_velocity_req.disable_axis.x = false;
		body_velocity_req.disable_axis.y = false;
		body_velocity_req.disable_axis.z = false;
		body_velocity_req.disable_axis.roll = true;
		body_velocity_req.disable_axis.pitch = true;
		body_velocity_req.disable_axis.yaw = false;
		
		// TO BE COMMENTED --> only for debug purposes
//		if (body_velocity_req.twist.linear.x != 0.0) 
//			body_velocity_req.disable_axis.x = false;
//		else 
//			body_velocity_req.disable_axis.x = true;
//
//		if (body_velocity_req.twist.linear.y != 0.0) 
//			body_velocity_req.disable_axis.y = false;
//		else 
//			body_velocity_req.disable_axis.y = true;
//
//		if (body_velocity_req.twist.linear.z != 0.0) 
//			body_velocity_req.disable_axis.z = false;
//		else 
//			body_velocity_req.disable_axis.z = true;
//
//		if (body_velocity_req.twist.angular.z != 0.0) 
//			body_velocity_req.disable_axis.yaw = false;
//		else 
//			body_velocity_req.disable_axis.yaw = true;
		// <--

		if((body_velocity_req.twist.linear.x == 0.0) &&
		   (body_velocity_req.twist.linear.y == 0.0) &&
		   (body_velocity_req.twist.linear.z == 0.0) &&
		   (body_velocity_req.twist.angular.z == 0.0)){
				body_velocity_req.disable_axis.x = true;
				body_velocity_req.disable_axis.y = true;
				body_velocity_req.disable_axis.z = true;
				body_velocity_req.disable_axis.yaw = true;
				body_velocity_req.goal.priority = auv_msgs::GoalDescriptor::PRIORITY_LOW;
		}
		_pub_body_velocity_req.publish(body_velocity_req);
	}


	/**
	 * This method is a callback of an internal timer.
	 * Every time it's call checks if in one second we have received the correct ack_msg.
	 * If the message is not correctly received it cleans the setpoints and setup everynthing to 0 to stop the robot.
	 *
	 * @param event Is not used. ROS must use it in this kind of callbacks
	 *
	 * @author Arnau Carrera
	 * @date 22/05/2011
	 */
	void checkJoystick ( const ros::TimerEvent& e )
	{
		if (_joystick_init){
			if ( _joystick_alive ) {
				_joystick_alive = false ;
			}
			else {
				ROS_FATAL( "WE HAVE LOST THE Joystick!!!!!!" );
				auv_msgs::BodyVelocityReq body_velocity_req;
	
				//header
				body_velocity_req.header.stamp = ros::Time::now() ;
	
				//goal
				body_velocity_req.goal.priority = auv_msgs::GoalDescriptor::PRIORITY_LOW;
				body_velocity_req.goal.requester = "/control_g500/teleoperation" ;
	
				//disabled_axis
				body_velocity_req.disable_axis.x = true;
				body_velocity_req.disable_axis.y = true;
				body_velocity_req.disable_axis.z = true;
				body_velocity_req.disable_axis.roll = true;
				body_velocity_req.disable_axis.pitch = true;
				body_velocity_req.disable_axis.yaw = true;
	
				//twist set-point
				body_velocity_req.twist.linear.x = 0.0;
				body_velocity_req.twist.linear.y = 0.0;
				body_velocity_req.twist.linear.z = 0.0;
				body_velocity_req.twist.angular.x = 0.0;
				body_velocity_req.twist.angular.y = 0.0;
				body_velocity_req.twist.angular.z = 0.0;
	
				_pub_body_velocity_req.publish(body_velocity_req);
			}
		}
		else {
			ROS_INFO("Waiting for the joystick...");
		}
		
		std_msgs::String msg ;
		msg.data = boost::lexical_cast<std::string>(_seq) + " ok" ;
		_pub_check_joystick.publish( msg ) ;
		
	}


	/**
	 * This method is a callback of ack_msg published.
	 * Every time it's call check if ack msg is the answer of the previous ok_msg.
	 *
	 * @param event Is not used. Ros must use in this kind of callbacks
	 *
	 * @author Arnau Carrera
	 * @date 22/05/2011
	 */
	void joystickAckCallback ( const std_msgs::String& ack_msg  )
	{
		std::istringstream iss( ack_msg.data ) ;
		std::string msg;
		unsigned int seq ;
		iss >> seq ;
		iss >> msg ;
		if ( "ack" == msg && seq == _seq+1 ) {
			_joystick_alive = true;
			_joystick_init = true;
			++_seq ;
		}
	}
};

int main(int argc, char **argv)
{
	ros::init(argc, argv, "teleoperation");
	Teleoperation teleoperation;
	teleoperation.init();
	ros::spin() ;
	return 0;
}
