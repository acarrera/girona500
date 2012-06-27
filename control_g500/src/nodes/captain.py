#!/usr/bin/env python

# ROS imports
import roslib 
roslib.load_manifest('control_g500')
import rospy
import actionlib

from threading import Thread

from auv_msgs.msg import *
from control_g500.msg import *
from control_g500.srv import GotoSrv, GotoSrvRequest, GotoSrvResponse
from safety_g500.msg import MissionStatus
from std_srvs.srv import Empty, EmptyResponse, EmptyRequest
from safety_g500.srv import MissionTimeout, MissionTimeoutRequest, MissionTimeoutResponse


class Trajectory:
    def __init__(self, north, east, depth, altitude, altitude_mode, roll, pitch, yaw, wait, disable_axis, tolerance, priority):
        if len(north) == len(east) == len(depth) == len(altitude) == len(altitude_mode) == len(roll) == len(pitch) == len(yaw) == len(wait):
            self.north = north
            self.east = east
            self.depth = depth
            self.altitude = altitude
            self.altitude_mode = altitude_mode
            self.roll = roll
            self.pitch = pitch
            self.yaw = yaw
            self.wait = wait
            self.disable_axis = disable_axis
            self.tolerance = tolerance
            self.priority = priority
        else:
            rospy.logerr("Invalid trajectory parameters!")
            
    
class Captain:
    def __init__(self, name):
        self.name = name
        
        # Init internal state 
        self.init_trajectory = False
        self.init_keep_pose = False
        self.init_goto = False
        self.init_keep_on_surface = False
        
        # Create publisher
        self.pub_mission_status = rospy.Publisher("/control_g500/mission_status", MissionStatus)
        self.pub_body_velocity_req = rospy.Publisher("/control_g500/body_velocity_req", BodyVelocityReq)
        
        # Load config parameters
        self.getConfig()
        
        # Create mission status messager
        self.mission_status = MissionStatus()
        
        # Create "absolute" client
        self.client_absolute = actionlib.SimpleActionClient('absolute_movement', WorldWaypointReqAction)
        self.client_absolute.wait_for_server()
    
        # Create Timer
        rospy.Timer(rospy.Duration(1.0), self.pubMissionStatus)
        
        # Create Subscriber
        rospy.Subscriber("/navigation_g500/nav_sts", NavSts, self.updateNavSts)
        self.nav = NavSts()
        
        # Create services
        self.enable_trajectory = rospy.Service('/control_g500/enable_trajectory', Empty, self.enableTrajectory)
        self.disable_trajectory = rospy.Service('/control_g500/disable_trajectory', Empty, self.disableTrajectory)
        self.enable_keep_position = rospy.Service('/control_g500/enable_keep_position', Empty, self.enableKeepPosition)
        self.disable_keep_position = rospy.Service('/control_g500/disable_keep_position', Empty, self.disableKeepPosition)
        self.goto = rospy.Service('/control_g500/goto', GotoSrv, self.goto)
        self.enable_keep_on_surface = rospy.Service('/control_g500/enable_keep_on_surface', Empty, self.enableKeepOnSurface)
        self.disable_keep_on_surface = rospy.Service('/control_g500/disable_keep_on_surface', Empty, self.disableKeepOnSurface)
        
        # Map shutdown function
        rospy.on_shutdown(self.stop)
        
         
    def enableKeepOnSurface(self, req):
        if not self.init_keep_pose and not self.init_trajectory and not self.init_goto and not self.init_keep_on_surface:
            self.init_keep_on_surface = True
            t = Thread(target=self.keepOnSurfaceThrd, args=())
            t.start()
            
        return EmptyResponse()
    
    
    def disableKeepOnSurface(self, req):
        rospy.loginfo('%s Disable keep on surface', self.name)
        self.init_keep_on_surface = False
        return EmptyResponse()
    
    
    def keepOnSurfaceThrd(self):
        bvr = BodyVelocityReq()
        bvr.twist.linear.x = 0.0
        bvr.twist.linear.y = 0.0
        bvr.twist.linear.z = -0.05
        bvr.twist.angular.x = 0.0
        bvr.twist.angular.y = 0.0
        bvr.twist.angular.z = 0.0
        bvr.disable_axis.x = True
        bvr.disable_axis.y = True
        bvr.disable_axis.z = False
        bvr.disable_axis.roll = True
        bvr.disable_axis.pitch = True
        bvr.disable_axis.yaw = True
        bvr.goal.priority = GoalDescriptor.PRIORITY_NORMAL
        bvr.goal.requester = self.name
        
        while self.init_keep_on_surface:
            bvr.header.stamp = rospy.Time.now()
            self.pub_body_velocity_req.publish(bvr)
            rospy.sleep(0.1)
    
    
    def enableKeepPosition(self, req):
        if self.init_keep_on_surface:
            self.disableKeepOnSurface(EmptyRequest())
            
        if not self.init_keep_pose and not self.init_trajectory and not self.init_goto:
            self.init_keep_pose = True
            rospy.loginfo('%s Enable keep position', self.name)
            goal = WorldWaypointReqGoal()
            goal.goal.requester = self.name
            goal.goal.id = 1
            goal.goal.priority = GoalDescriptor.PRIORITY_NORMAL
            goal.altitude_mode = False
            goal.position.north = self.nav.position.north
            goal.position.east = self.nav.position.east
            goal.position.depth = self.nav.position.depth
            goal.altitude = self.nav.altitude
            goal.orientation.roll = self.nav.orientation.roll
            goal.orientation.pitch = self.nav.orientation.pitch
            goal.orientation.yaw = self.nav.orientation.yaw
            
            # Here we choose the movement type --> XYZYaw, neverending
            goal.mode = 'neverending'
            goal.disable_axis.x = False
            goal.disable_axis.y = False
            goal.disable_axis.z = False
            goal.disable_axis.roll = True
            goal.disable_axis.pitch = True
            goal.disable_axis.yaw = False
            
            # Set tolerance
            goal.position_tolerance.x = 0.01
            goal.position_tolerance.y = 0.01
            goal.position_tolerance.z = 0.01
            goal.orientation_tolerance.roll = 0.01
            goal.orientation_tolerance.pitch = 0.01
            goal.orientation_tolerance.yaw = 0.01
            self.client_absolute.send_goal(goal)
        else:
            rospy.logerr('%s: unable to execute keepPosition, another service is running.', self.name)
            rospy.logerr('%s: keepPose: %s', self.name, self.init_keep_pose)
            rospy.logerr('%s: trajectory: %s', self.name, self.init_trajectory)
            rospy.logerr('%s: goto: %s', self.name, self.init_goto)
        
        return EmptyResponse()
    
        
    def disableKeepPosition(self, req):
        rospy.loginfo('%s Disable keep position', self.name)
        self.init_keep_pose = False
        self.client_absolute.cancel_goal()
        return EmptyResponse()
    
    
    def enableTrajectory(self, req):
        """Follows a list of way-points using the LOS algorithm. 
           Once each way-point is reached, if some waiting time is defined,
           performs a keep position movement to this way-point. """
        
        if self.init_keep_on_surface:
            self.disableKeepOnSurface(EmptyRequest())
            
        if not self.init_trajectory and not self.init_keep_pose and not self.init_goto:
            self.init_trajectory = True
            
            try: 
                rospy.wait_for_service('/mission_timeout', 5)
                # TODO: solve wait for android services
                rospy.sleep(1.0)
                self.mission_timeout_srv = rospy.ServiceProxy('/mission_timeout', MissionTimeout)
                mt = MissionTimeoutRequest()
                mt.start_mission = True
                mt.time_out = self.trajectory_time_out
                rospy.loginfo('Init trajectory with mission_timeout: %s', mt)
                response = self.mission_timeout_srv(mt)
                if not response.success:
                    rospy.logerr('%s, Error initializing Mission Timeout', self.name)
            except rospy.exceptions.ROSException:
                rospy.logerr('%s, Error initializing Mission Timeout. Are you in simulation mode?', self.name)
            
            t = Thread(target=self.followTrajectory, args=())
            t.start()
        else:
            rospy.logerr('%s, Impossible to enable trajectory, another service is running.', self.name)
            rospy.logerr('%s: keepPose: %s', self.name, self.init_keep_pose)
            rospy.logerr('%s: trajectory: %s', self.name, self.init_trajectory)
            rospy.logerr('%s: goto: %s', self.name, self.init_goto)
        
        return EmptyResponse()
    
    
    def disableTrajectory(self, req):
        rospy.loginfo("%s: Disable trajectory", self.name)    
        self.init_trajectory = False
        self.client_absolute.cancel_goal()
        return EmptyResponse()
    
    
    def goto(self, req):
        
        if self.init_keep_on_surface:
            self.disableKeepOnSurface(EmptyRequest())
            
        if not self.init_trajectory and not self.init_keep_pose and not self.init_goto:
            self.init_goto = True
            rospy.loginfo('north: %s', req.north)
            rospy.loginfo('east: %s', req.east)
            rospy.loginfo('z: %s', req.z)
            rospy.loginfo('altitude_mode: %s', req.altitude_mode)
            rospy.loginfo('%s Enable keep position', self.name)
            
            goal = WorldWaypointReqGoal()
            goal.goal.requester = self.name
            goal.goal.id = 1
            goal.goal.priority = GoalDescriptor.PRIORITY_NORMAL
            goal.altitude_mode = req.altitude_mode
            goal.position.north = req.north
            goal.position.east = req.east
            goal.position.depth = req.z
            goal.altitude = req.z
            goal.orientation.roll = 0.0
            goal.orientation.pitch = 0.0
            goal.orientation.yaw = 0.0
            
            goal.mode = 'waypoint'
            goal.disable_axis.x = False
            goal.disable_axis.y = True
            goal.disable_axis.z = False
            goal.disable_axis.roll = True
            goal.disable_axis.pitch = True
            goal.disable_axis.yaw = False
            
            # Set tolerance
            tolerance = 2.0
            goal.position_tolerance.x = tolerance
            goal.position_tolerance.y = tolerance
            goal.position_tolerance.z = tolerance/2.0
            goal.orientation_tolerance.roll = tolerance
            goal.orientation_tolerance.pitch = tolerance
            goal.orientation_tolerance.yaw = tolerance
            
            self.client_absolute.send_goal(goal)
            self.client_absolute.wait_for_result()
        else:
            rospy.logerr('%s, Impossible to enable trajectory, another service is running.', self.name)
            rospy.logerr('%s: keepPose: %s', self.name, self.init_keep_pose)
            rospy.logerr('%s: trajectory: %s', self.name, self.init_trajectory)
            rospy.logerr('%s: goto: %s', self.name, self.init_goto)
        
        self.init_goto = False
        return GotoSrvResponse(True)
    
    
    def followTrajectory(self):
        for i in range(len(self.trajectory.north)):
            # Creates a goal to send to the action server.
            goal = WorldWaypointReqGoal()
            goal.goal.requester = self.name
            
            # It is important that the first goal.id in a trajectory is 0 (pilot requirement)
            goal.goal.id = i 
            
            goal.goal.priority = self.trajectory.priority
            goal.altitude_mode = self.trajectory.altitude_mode[i]
            goal.position.north = self.trajectory.north[i]
            goal.position.east = self.trajectory.east[i]
            goal.position.depth = self.trajectory.depth[i]
            goal.altitude = self.trajectory.altitude[i]
            goal.orientation.roll = self.trajectory.roll[i]
            goal.orientation.pitch = self.trajectory.pitch[i]
            goal.orientation.yaw = self.trajectory.yaw[i]
            
            # Here we choose the movement type. From disable_axis we can choose between
            # waypoint and LOS. With goal.mode, we choose LOS
            goal.mode = 'los'
            goal.disable_axis.x = False
            goal.disable_axis.y = True
            goal.disable_axis.z = self.trajectory.disable_axis[2]
            goal.disable_axis.roll = True
            goal.disable_axis.pitch = True
            goal.disable_axis.yaw = False
            # -------------------------------------------
            
            goal.position_tolerance.x = self.trajectory.tolerance[0]
            goal.position_tolerance.y = self.trajectory.tolerance[1]
            goal.position_tolerance.z = self.trajectory.tolerance[2]
            goal.orientation_tolerance.roll = self.trajectory.tolerance[3]
            goal.orientation_tolerance.pitch = self.trajectory.tolerance[4]
            goal.orientation_tolerance.yaw = self.trajectory.tolerance[5]
        
            #Fill MissionStatus topic
            self.mission_status.current_wp = i + 1
            self.mission_status.total_wp = len(self.trajectory.north)
            self.mission_status.wp_north = self.trajectory.north[i]
            self.mission_status.wp_east = self.trajectory.east[i]

            self.mission_status.altitude_mode = self.trajectory.altitude_mode[i] 
            if self.trajectory.altitude_mode[i]: 
                self.mission_status.wp_depth_altitude = self.trajectory.altitude[i]
            else: 
                self.mission_status.wp_depth_altitude = self.trajectory.depth[i]
            self.mission_status.wp_remaining_time = self.trajectory.wait[i]
            
            # Send the waypoint
            rospy.loginfo("%s, Request GOAL:\n%s", self.name, goal)
            self.client_absolute.send_goal(goal)
            
            # time out for each way-point?
            self.client_absolute.wait_for_result()
            result = self.client_absolute.get_result()
            rospy.loginfo("%s, Obtained RESULT:\n%s", self.name, result)
            
            # Check if the trajectory has been disabled
            if not self.init_trajectory:
                rospy.loginfo("%s: Trajectory disabled", self.name)
                return EmptyResponse()
            
            #Check if the vehicle has to keep this way-point for a while
            if self.trajectory.wait[i] > 0.0:
                # Here we choose the movement type --> neverending Keep Pose
                goal.mode = 'neverending'
                goal.disable_axis.x = False
                goal.disable_axis.y = False
                goal.disable_axis.z = self.trajectory.disable_axis[2]
                goal.disable_axis.roll = True
                goal.disable_axis.pitch = True
                goal.disable_axis.yaw = True
                
                # Set tolerance to 0.01
                goal.position_tolerance.x = 0.01
                goal.position_tolerance.y = 0.01
                goal.position_tolerance.z = 0.01
                goal.orientation_tolerance.roll = 0.01
                goal.orientation_tolerance.pitch = 0.01
                goal.orientation_tolerance.yaw = 0.01
                
                #Wait for n seconds
                rospy.loginfo("%s, Wait for %s seconds", self.name, self.trajectory.wait[i])
                self.client_absolute.send_goal(goal)
                
                for w in range(int(self.trajectory.wait[i])):
                    rospy.sleep(1.0)
                    # Update MissionStatus topic
                    self.mission_status.wp_remaining_time = self.trajectory.wait[i] - w
                    
                    #Check if the trajectory is aborted while waiting
                    if not self.init_trajectory:
                        rospy.loginfo("%s Trajectory disabled", self.name)
                        return EmptyResponse()
                
                rospy.loginfo("%s, Waiting done %s", self.name, self.trajectory.wait[i])
        
                #Abort the action when the time has expired
                self.client_absolute.cancel_goal()    
                rospy.loginfo("%s, Finalize KeepPose action", self.name)
                
        # When the trajectory finalizes mark it and return the service        
        self.init_trajectory = False    

    
    def pubMissionStatus(self, event):
        self.pub_mission_status.publish(self.mission_status)
    
    
    def updateNavSts(self, nav_sts):
        self.nav = nav_sts
        self.mission_status.current_north = nav_sts.position.north
        self.mission_status.current_east = nav_sts.position.east
        self.mission_status.current_altitude = nav_sts.altitude
        self.mission_status.current_depth = nav_sts.position.depth
    
        
    def stop(self):
        rospy.loginfo("%s: Cancel the current goal", self.name)
        self.init_trajectory = False
        self.init_keep_pose = False
        self.init_goto = False
        self.init_keep_on_surface = False
        self.client_absolute.cancel_goal()
         
         
    def getConfig(self):
        if rospy.has_param("trajectory/north") :
            north = rospy.get_param("trajectory/north")
        else:
            rospy.logfatal("trajectory/north")
        
        if rospy.has_param("trajectory/east") :
            east = rospy.get_param("trajectory/east")
        else:
            rospy.logfatal("trajectory/east")
        
        if rospy.has_param("trajectory/depth") :
            depth = rospy.get_param("trajectory/depth")
        else:
            rospy.logfatal("trajectory/depth")
        
        if rospy.has_param("trajectory/altitude") :
            altitude = rospy.get_param("trajectory/altitude")
        else:
            rospy.logfatal("trajectory/altitude")
        
        if rospy.has_param("trajectory/altitude_mode") :
            altitude_mode = rospy.get_param("trajectory/altitude_mode")
        else:
            rospy.logfatal("trajectory/altitude_mode")
            
        if rospy.has_param("trajectory/roll") :
            roll = rospy.get_param("trajectory/roll")
        else:
            rospy.logfatal("trajectory/roll")
        
        if rospy.has_param("trajectory/pitch") :
            pitch = rospy.get_param("trajectory/pitch")
        else:
            rospy.logfatal("trajectory/pitch")
        
        if rospy.has_param("trajectory/yaw") :
            yaw = rospy.get_param("trajectory/yaw")
        else:
            rospy.logfatal("trajectory/yaw")
        
        if rospy.has_param("trajectory/wait") :
            wait = rospy.get_param("trajectory/wait")
        else:
            rospy.logfatal("trajectory/wait")
        
        if rospy.has_param("trajectory/disable_axis") :
            disable_axis = rospy.get_param("trajectory/disable_axis")
        else:
            rospy.logfatal("trajectory/disable_axis")
        
        if rospy.has_param("trajectory/tolerance") :
            tolerance = rospy.get_param("trajectory/tolerance")
        else:
            rospy.logfatal("trajectory/tolerance")
       
        if rospy.has_param("trajectory/priority") :
            priority = rospy.get_param("trajectory/priority")
        else:
            rospy.logfatal("trajectory/priority")
        
        if rospy.has_param("trajectory/time_out") :
            self.trajectory_time_out = rospy.get_param("trajectory/time_out")
        else:
            rospy.logfatal("trajectory/time_out")
        
        if rospy.has_param("captain/enable_keep_on_surface") :
            init_keep_on_surface = rospy.get_param("captain/enable_keep_on_surface")
            rospy.loginfo("captain/enable_keep_on_surface = %s", init_keep_on_surface)
            if init_keep_on_surface:
                self.enableKeepOnSurface(EmptyRequest())
        else:
            rospy.loginfo("captain/enable_keep_on_surface = False")
            
            
        self.trajectory = Trajectory(north, 
                                     east, 
                                     depth, 
                                     altitude, 
                                     altitude_mode, 
                                     roll, 
                                     pitch, 
                                     yaw, 
                                     wait, 
                                     disable_axis, 
                                     tolerance, 
                                     priority)


if __name__ == '__main__':
    try:
         rospy.init_node('captain')
         follow_trajectory = Captain(rospy.get_name())
         rospy.spin()
    except rospy.ROSInterruptException: pass
