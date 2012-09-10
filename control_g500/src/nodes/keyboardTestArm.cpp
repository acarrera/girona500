/*
 * keyboard_vel.cpp
 *
 *  Created on: 22/05/2011
 *  updated on: 11/01/2012
 *      Author: arnau & narcis
 */

//TODO: MIRAR EL CODI QUE HI HA A: "teleop_pr2_keyboard.cpp" del paquet standard del PR2 de ROS

#include <boost/thread/mutex.hpp>
#include <termios.h>
#include <stdio.h>
#include <signal.h>

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sensor_msgs/Joy.h"
#include <boost/thread.hpp>
#include "auv_msgs/NavSts.h"
#include <math.h> 
#define _USE_MATH_DEFINES


//Test for ARM
#include "nav_msgs/Odometry.h"

struct JoyConfig {
	double min_x_v;
	double max_x_v;
	double inc_x;
	double min_y_v;
	double max_y_v;
	double inc_y;
	double min_z_v;
	double max_z_v;
	double inc_z;
	double min_yaw_v;
	double max_yaw_v;
	double inc_yaw;
};


const int _NUMBER_OF_SETPOINTS_ = 6;
JoyConfig _config;

std::vector< int > _pressed_keys ;
std::vector< float > _setpoints ;
//Keyboard
std::vector< float > _setpointsARM(3) ;
float _configIncArm = 0.01;
float _maxArm = 0.5 ;
float _minArm = -0.5 ;

boost::mutex _setpoints_mutex;
boost::shared_ptr< boost::thread > _reading_thread ;

bool _reading_on = false ;
bool _shutdown_thread = false;
struct termios oldt, newt ;
ros::Publisher _pub_ack_teleop ;
ros::Publisher _pub_command_odom ;


/**
 * Get the configuration from an xml file. The data is set in xmlMapper.
 * Like in the previous COLA2
 *
 * @author Arnau Carrera
 * @date 22/05/2011
 */
void getConfig() ;


/**
 *This method configure the standard input to read the key pressed
 *and after reading sets the configuration again.
 *
 * @author Arnau Carrera
 * @date 22/05/2011
 */
int getChar() ;


/**
 *This function is executed in a independent thread.
 *It reads the key pressed using gethChar and do the action
 *needed in the setpoints vector
 *
 * @author Arnau Carrera
 * @date 22/05/2011
 */
void readKeyboardHits( ) ;


/**
 *This function is executed in every loop_rate in the main.
 *It generates the message to send to the teleoperation and
 *publish it.
 *
 * @author Arnau Carrera
 * @date 22/05/2011
 */
void publishSetpoints( ros::Publisher& pub ) ;


/**
 *This function is a callback.
 *This callback is called when teleoperation publish a
 *joystick_ok( this message contains seq_num and ok ).
 *The function read this message and answer with ack message.
 *
 * @param pub The publisher have to be the type JoystickData
 *
 * @author Arnau Carrera
 * @date 22/05/2011
 */
void joystickOkCallback(const std_msgs::String& ok_msg);

void navStsCallback( const auv_msgs::NavSts& msg );

/**
 *This functions was called when you recived a SIGTERM event.
 *When you received this, the method recover the keyboard.
 * @param ok_msg This is a string with num_seq + "ok"
 * @author Arnau Carrera
 * @date 22/05/2011
 */
void quit(int sig) ;


/**
 * Initialize all the class, and have a loop_rate where it calls the publish method.
 * @author Arnau Carrera
 * @date 22/05/2011
 */
int main(int argc, char **argv);


int
main(int argc, char **argv)
{
	ros::init(argc, argv, "keyboard_test_arm", ros::init_options::NoSigintHandler ) ;

	// BEGINING INITIALIZE
	getConfig();
	ROS_INFO("Configuration Loaded correctly") ;
	_pressed_keys.push_back( 1 ) ;
	_setpoints.resize( _NUMBER_OF_SETPOINTS_ ) ;

	//Millor Així o no val la pena tenir dos Threads.
	_reading_thread = boost::shared_ptr<boost::thread>(new boost::thread(boost::bind(&readKeyboardHits)));
	ROS_INFO("Initialized thread correctly") ;
	
	// END INITIALIZE

	ros::NodeHandle n;

	//Publishers
	ros::Publisher _pub = n.advertise<sensor_msgs::Joy>("/control_g500/joystick_data", 1);
	_pub_ack_teleop = n.advertise<std_msgs::String >("/control_g500/joystick_ack", 1);
        _pub_command_odom = n.advertise<nav_msgs::Odometry>("/arm/desired_pose",1) ;

	//Subcribers
	ros::Subscriber sub_ok = n.subscribe( "/control_g500/joystick_ok", 1, joystickOkCallback);

    signal(SIGINT, quit);

	ros::Rate loop_rate( 10.0 ) ;
	_reading_on = true ;

	while ( ros::ok() )
	{
		ros::spinOnce();
		publishSetpoints(_pub);
		loop_rate.sleep() ;
	}
	
	_shutdown_thread = true;
	
	return 0;
}



void
getConfig(){
	if(!ros::param::getCached("joy/min_x_v", _config.min_x_v)) {ROS_FATAL("Invalid parameters for joy/min_x_v in param server!"); ros::shutdown();}
	if(!ros::param::getCached("joy/max_x_v", _config.max_x_v)) {ROS_FATAL("Invalid parameters for joy/max_x_v in param server!"); ros::shutdown();}
	if(!ros::param::getCached("joy/x_inc", _config.inc_x)) {ROS_FATAL("Invalid parameters for joy/x_inc in param server!"); ros::shutdown();}
	if(!ros::param::getCached("joy/min_y_v", _config.min_y_v)) {ROS_FATAL("Invalid parameters for joy/min_y_v in param server!"); ros::shutdown();}
	if(!ros::param::getCached("joy/max_y_v", _config.max_y_v)) {ROS_FATAL("Invalid parameters for joy/max_y_v in param server!"); ros::shutdown();}
	if(!ros::param::getCached("joy/y_inc", _config.inc_y)) {ROS_FATAL("Invalid parameters for joy/y_inc in param server!"); ros::shutdown();}
	if(!ros::param::getCached("joy/min_z_v", _config.min_z_v)) {ROS_FATAL("Invalid parameters for joy/min_z_v in param server!"); ros::shutdown();}
	if(!ros::param::getCached("joy/max_z_v", _config.max_z_v)) {ROS_FATAL("Invalid parameters for joy/max_z_v in param server!"); ros::shutdown();}
	if(!ros::param::getCached("joy/z_inc", _config.inc_z)) {ROS_FATAL("Invalid parameters for joy/z_inc in param server!"); ros::shutdown();}
	if(!ros::param::getCached("joy/min_yaw_v", _config.min_yaw_v)) {ROS_FATAL("Invalid parameters for joy/min_yaw_v in param server!"); ros::shutdown();}
	if(!ros::param::getCached("joy/max_yaw_v", _config.max_yaw_v)) {ROS_FATAL("Invalid parameters for joy/max_yaw_v in param server!"); ros::shutdown();}
	if(!ros::param::getCached("joy/yaw_inc", _config.inc_yaw)) {ROS_FATAL("Invalid parameters for joy/yaw_inc in param server!"); ros::shutdown();}
}



int
getChar()
{
	int ch ;
	tcgetattr( STDIN_FILENO, &oldt ) ;
	//newt = oldt ;
	memcpy(&newt, &oldt, sizeof(struct termios));
	newt.c_lflag &= ~( ICANON | ECHO ) ;
	newt.c_cc[VEOL] = 1;
	newt.c_cc[VEOF] = 2;
	tcsetattr( STDIN_FILENO, TCSANOW, &newt ) ;
	ch = getchar() ;
	tcsetattr( STDIN_FILENO, TCSANOW, &oldt ) ;
	return ch ;
}



void
readKeyboardHits( ) {

	while(!_shutdown_thread)
	{
		const size_t X = 0 ;
		const size_t Y = 1 ;
		const size_t Z = 2 ;
//		const size_t Roll = 3 ;
//		const size_t Pitch = 4 ;
		const size_t YAW = 5 ;

		//Forward, backward, turn left and turn right
		const int KEY_W = 87 ;
		const int KEY_w = 119 ;
		const int KEY_S = 83 ;
		const int KEY_s = 115 ;
		const int KEY_A = 65 ;
		const int KEY_a = 97 ;
		const int KEY_D = 68 ;
		const int KEY_d = 100 ;

		//Up and down
		const int KEY_UP = 65 ;
		const int KEY_LEFT = 68 ;
		const int KEY_RIGHT = 67 ;
		const int KEY_DOWN = 66 ;

		//Control of the Arm
		//X
		const int KEY_I = 73 ;
		const int KEY_i = 105 ;
		const int KEY_K = 75 ;
		const int KEY_k = 107 ; 
		//Y
		const int KEY_O = 79 ;
		const int KEY_o = 111 ;
		const int KEY_L = 76 ;
		const int KEY_l = 108 ;
		//Z
		const int KEY_P = 80 ;
		const int KEY_p = 112 ;
		const int KEY_N = 165 ;
		const int KEY_n = 164 ;



		//Esc and Space control actions
		const int KEY_ESC = 27 ;
		const int KEY_OBRACKET = 91 ;
		const int KEY_SPACE = 32 ;


		int key = getChar() ;

		_pressed_keys.push_back( key ) ;

		boost::mutex::scoped_lock lock( _setpoints_mutex ) ;

		if ( key == KEY_ESC ) {

			if ( getChar() == KEY_OBRACKET ) {
				int arrow_key = getChar() ;
				if ( arrow_key == KEY_UP ) {
					_setpoints[ Z ] -= _config.inc_z;
				}
				else if ( arrow_key == KEY_LEFT ) {
					_setpoints[ Y ] -= _config.inc_y;
				}
				else if ( arrow_key == KEY_RIGHT ) {
					_setpoints[ Y ] += _config.inc_y;
				}
				else if ( arrow_key == KEY_DOWN ) {
					_setpoints[ Z ] += _config.inc_z;
				}
			}
		}
		else if ( key == KEY_W || key == KEY_w ) {
			_setpoints[ X ] += _config.inc_x;
		}
		else if ( key == KEY_S || key == KEY_s ) {
			_setpoints[ X ] -= _config.inc_x;
		}
		else if ( key == KEY_D || key == KEY_d ) {
			_setpoints[YAW] += _config.inc_yaw;
		}
		else if ( key == KEY_A || key == KEY_a ) {
			_setpoints[YAW] -= _config.inc_yaw;
		}
		// Keys ARM
		else if ( key == KEY_I || key == KEY_i ) {
		  _setpointsARM[ 0 ] += _configIncArm;
		}
		else if ( key == KEY_K || key == KEY_k ) {
			_setpointsARM[ 0 ] -= _configIncArm;
		}
		else if ( key == KEY_O || key == KEY_o ) {
			_setpointsARM[ 1 ] += _configIncArm;
		}
		else if ( key == KEY_L || key == KEY_l ) {
			_setpointsARM[ 1 ] -= _configIncArm;
		}
		else if ( key == KEY_P || key == KEY_p ) {
			_setpointsARM[ 2 ] += _configIncArm;
		}
		else if ( key == KEY_N || key == KEY_n ) {
			_setpointsARM[ 2 ] -= _configIncArm;
		}
		else if ( key == KEY_SPACE ) {
			//std::fill( _setpoints.begin(), _setpoints.end(), 0.0 );
			_setpoints[X] = 0.0;
			_setpoints[Y] = 0.0;
			_setpoints[Z] = 0.0;
			_setpoints[YAW] = 0.0;

			//arm
			_setpointsARM[0] = 0.0;
			_setpointsARM[1] = 0.0;
			_setpointsARM[2] = 0.0;
		}
		else {
			ROS_INFO( "Keycode is: %d. Ignored Key.", key );
		}

		//Saturate
		if(_setpoints[X] > _config.max_x_v) _setpoints[X] = _config.max_x_v;
		else if(_setpoints[X] < _config.min_x_v) _setpoints[X] = _config.min_x_v;
		 
		if(_setpoints[Y] > _config.max_y_v) _setpoints[Y] = _config.max_y_v;
		else if(_setpoints[Y] < _config.min_y_v) _setpoints[Y] = _config.min_y_v;
				
		if(_setpoints[Z] > _config.max_z_v) _setpoints[Z] = _config.max_z_v;
		else if(_setpoints[Z] < _config.min_z_v) _setpoints[Z] = _config.min_z_v;

		if(_setpoints[YAW] > _config.max_yaw_v) _setpoints[YAW] = _config.max_yaw_v;
		else if(_setpoints[YAW] < _config.min_yaw_v) _setpoints[YAW] = _config.min_yaw_v;	
		// Saturate ARM
		if(_setpointsARM[0] > _maxArm) _setpointsARM[0] = _maxArm;
		else if(_setpointsARM[0] < _minArm) _setpointsARM[0] = _minArm;
		 
		if(_setpointsARM[1] > _maxArm) _setpointsARM[2] = _maxArm;
		else if(_setpointsARM[1] < _minArm) _setpointsARM[2] = _minArm;
				
		if(_setpointsARM[2] > _maxArm) _setpointsARM[2] = _maxArm;
		else if(_setpointsARM[2] < _minArm) _setpointsARM[2] = _minArm;

		nav_msgs::Odometry odom ;
		odom.pose.pose.position.x = _setpointsARM[0];
		odom.pose.pose.position.y = _setpointsARM[1];
		odom.pose.pose.position.z = _setpointsARM[2];
		_pub_command_odom.publish(odom);
	}
}


void
publishSetpoints( ros::Publisher& pub )
{
	boost::mutex::scoped_lock lock( _setpoints_mutex ) ;

	sensor_msgs::Joy msg ;
	msg.axes.reserve(_setpoints.size());
	copy(_setpoints.begin(), _setpoints.end(), back_inserter(msg.axes));

	msg.buttons.reserve(_pressed_keys.size());
	copy(_pressed_keys.begin(),_pressed_keys.end(),back_inserter(msg.buttons));

	msg.header.stamp = ros::Time::now() ;
	pub.publish( msg ) ;

	_pressed_keys.clear() ;
	_pressed_keys.resize(1) ;
}


void
joystickOkCallback( const std_msgs::String& ok_msg )
{
	std::istringstream iss( ok_msg.data ) ;
	std::string msg;
	unsigned int seq ;
	iss >> seq ;
	iss >> msg ;
	if ( "ok" == msg ) {
		std_msgs::String msg ;
		msg.data = boost::lexical_cast<std::string>(seq+1) + " ack" ;
		_pub_ack_teleop.publish( msg ) ;
	}
}


void
quit(int sig)
{
	tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
	ros::shutdown();
	exit(0);
}
