/*
 * arm_controller.cpp
 *
 *  Created on: 05/09/2012
 *      Author: arnau 
 */

#include "ros/ros.h"
#include <kdl_parser/kdl_parser.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainiksolver.hpp>
#include <kdl/chainiksolverpos_nr.hpp>
#include <kdl/chainiksolvervel_pinv.hpp>
#include <kdl/frames_io.hpp>
#include <kdl/frames.hpp>
#include <urdf/model.h>
#include "sensor_msgs/JointState.h"
#include "nav_msgs/Odometry.h"



class Arm_controller {

private:
  ros::NodeHandle _n ;
  ros::Subscriber _sub_joint_state  ;
  ros::Subscriber _sub_desired_pose ;
  ros::Publisher _pub_joint_command ;
  KDL::Tree _tree ;
  KDL::Chain _chain ;
  KDL::JntArray _jointpositions ;
  nav_msgs::Odometry _odom ;
  
  //  KDL::ChainFkSolverPos_recursive _fksolver ;
  //  KDL::ChainIkSolverVel_pinv _iksolverVel ;
  // KDL::ChainIkSolverPos_NR _iksolver ;

public:
  
  Arm_controller(){}

  ~Arm_controller() {}

  /**
   * This method initialize the transformation structure, reading from the urdf file and generate 
   * the forward kinematics and inverte kinematics. 
   *
   * @param
   *
   * @return True if all works correctly
   * @author Arnau Carrera
   * @data 7/09/2012
   */
  bool init()
  { 
    urdf::Model my_model;
    //TODO::change to a realtive file
    if (!my_model.initFile("/home/arnau/ROS/simG500/girona500/data2/scenes2/g500ARMPandora.urdf")) {
      ROS_ERROR("Failed to parse urdf robot model");
      return false;
    }
    if (!kdl_parser::treeFromUrdfModel(my_model, _tree)) {
      ROS_ERROR("Failed to construct kdl tree");
      return false;
    }

    _pub_joint_command = _n.advertise<sensor_msgs::JointState>("/uwsim/joint_state_command", 1);

    _sub_joint_state = _n.subscribe("/uwsim/joint_state", 1, &Arm_controller::jointCallback, this);
    _sub_desired_pose = _n.subscribe("/arm/desired_pose", 1, &Arm_controller::desiredPoseCallback, this);

    //TODO::change to configuration file the base and support
    if ( !_tree.getChain("base", "support", _chain) ) {
      ROS_ERROR( "We can not get the chain from the tree");
      return false;
    }
    // Create joint array
    _jointpositions = KDL::JntArray(_chain.getNrOfJoints());

    //_newPosition = false ;

    return true;
  }



  KDL::Tree getTree(){
    return _tree;
  }

  KDL::Chain getChain(){
    return _chain;
  }


  nav_msgs::Odometry getOdom(){
    return _odom;
  }


  KDL::JntArray getCurrentJoint(){
    return _jointpositions ;
  }  

  /**
   * This method is a callback for the updates of the status of the Status of the joint
   * @param event, is the message and the name of the publisher
   */
  void jointCallback(const ros::MessageEvent<sensor_msgs::JointState const>& event )
  {
    const std::string& publisher_name = event.getPublisherName();
    const sensor_msgs::JointState::ConstPtr& msg = event.getMessage();
    
    // ROS_INFO("JointState received from: [%s]", publisher_name.c_str());

    for (unsigned int i=0 ; i < _chain.getNrOfJoints() ; i++ ){
      _jointpositions(i) = msg->position[i]; 
    }
    
    ROS_INFO_STREAM( " The values are " << _jointpositions(0) << ", " << _jointpositions(1) << ", " << _jointpositions(2) << "\n" );
  }


  /**
   * This method is a callback to update the desired position
   *
   */
  void desiredPoseCallback(const ros::MessageEvent<nav_msgs::Odometry const>& event )
  {
    const std::string& publisher_name = event.getPublisherName();
    const nav_msgs::Odometry::ConstPtr& msg = event.getMessage();
    _odom = *msg; 
  }


  void publishJointState(const KDL::JntArray& joint) {
    sensor_msgs::JointState desiredJoint ;
    for (unsigned int i=0 ; i < _chain.getNrOfJoints() ; i++ ){
      desiredJoint.position[i] = joint(i) ;
    }
    _pub_joint_command.publish(desiredJoint) ;
  }

};


int main(int argc, char **argv)
{
  ros::init(argc, argv, "arm_controller");
  Arm_controller arm_controller;
  arm_controller.init();
  /*  KDL::Tree tree = arm_controller.getTree();
  ROS_INFO_STREAM( "Number of Joints: " << tree.getNrOfJoints() );
  ROS_INFO_STREAM( "Number of Segments: " << tree.getNrOfSegments() ); */

  KDL::Chain chain = arm_controller.getChain();
  // Forward Kinematics
  KDL::ChainFkSolverPos_recursive fksolver(chain);
  
  // Invert Kinematics
  //its neede a velocity and a fksolver
  //the ChainIkSolverPos_NR_JL can configure the join limits
  KDL::ChainIkSolverVel_pinv iksolverVel(chain);//Inverse velocity solver
  KDL::ChainIkSolverPos_NR iksolver(chain, fksolver, iksolverVel,100,1e-6);//Maximum 100 iterations, stop at accuracy 1e-6


  //final position of the joints
  KDL::JntArray desired_joints(chain.getNrOfJoints()) ;
  //initial position of the joints
  KDL::JntArray current_joints(chain.getNrOfJoints()) ; 


  while(ros::ok()){
    ros::spinOnce() ;
    nav_msgs::Odometry odom = arm_controller.getOdom() ;
    
    if ( odom.pose.pose.position.x != 0.0 || odom.pose.pose.position.y != 0.0 || odom.pose.pose.position.z != 0.0 ) {
      KDL::Frame current_frame;    
 
      if ( fksolver.JntToCart(arm_controller.getCurrentJoint(),current_frame) >= 0 ) {
	//desired_joints = current_joints ;
	ROS_INFO_STREAM( "Current Frame Pose " << current_frame.p[0] << ", " << current_frame.p[1] <<", " << current_frame.p[2]  ) ;
	current_frame.p[0] = current_frame.p[0] + odom.pose.pose.position.x ;
	current_frame.p[1] = current_frame.p[1] + odom.pose.pose.position.y ;
	current_frame.p[2] = current_frame.p[2] + odom.pose.pose.position.z ;
	ROS_INFO_STREAM( "Desired Frame Pose " << current_frame.p[0] << ", " << current_frame.p[1] <<", " << current_frame.p[2]  ) ;
	
	if ( iksolver.CartToJnt(arm_controller.getCurrentJoint(),current_frame,desired_joints) >=0  ) {
	  arm_controller.publishJointState(desired_joints);
	}
	else{
	  ROS_ERROR( "The KDL Library fail doing the inverse kinematics" );
	}

      }
      else {
	ROS_ERROR( "The KDL Library fail converting form Joints to frame" );
      }
    }

  }

  return 0;
}
