#!/usr/bin/env python

# Msgs imports
from auv_msgs.msg import *

# Python imports
from numpy import *
from math import *

# Custom imports
import cola2_lib

class MovementMode:
    ERROR = -1
    ABSOLUTE_X_Z_YAW = 0
    ABSOLUTE_X_Y_Z_YAW = 1
    RELATIVE_X_Y_Z_YAW = 2


class MoveMode:
    def __init__(self, now, pid_x_z_yaw, pid_x_y_z_yaw, pid_relative_x_y_z_yaw, max_velocity, min_velocity_los, max_angle_error):
        self.pid_x_z_yaw = pid_x_z_yaw
        self.pid_x_y_z_yaw = pid_x_y_z_yaw
        self.pid_relative_x_y_z_yaw = pid_relative_x_y_z_yaw
        self.max_velocity = max_velocity
        self.min_velocity_los = min_velocity_los
        self.max_angle_error = max_angle_error
        
        self.altitude = -1
        self.p = zeros(6)
        self.v = zeros(3)

        self.current_waypoint = {'active': False}
        self.pid_x_z_yaw.reset(now)
        self.pid_x_y_z_yaw.reset(now)
        self.pid_relative_x_y_z_yaw.reset(now)
        
        
    
    def computePosError(self, current_x, current_y, current_yaw, req_x, req_y):
        R = matrix(array([cos(current_yaw), -sin(current_yaw), 0.0, sin(current_yaw), cos(current_yaw), 0.0, 0.0, 0.0, 1.0]))
        R.resize(3, 3)
        
        p = matrix(array([current_x, current_y, 0.0]))
        p.resize(3, 1)
        
        aux = -R.T * p
        
        T = eye(4)
        T[0:3, 0:3] = R.T
        T[0, 3] = aux[0,0]
        T[1, 3] = aux[1,0]
        T[2, 3] = aux[2,0]
        
        p_req = matrix(array([req_x, req_y, 0.0, 1.0]))
        p_req.resize(4, 1)
        
        distance = T*p_req
        #print "Distance: " + str(distance)
        return [distance[0,0], distance[1,0]]
        
        
    def checkTolerance(self, b, current, desired, tolerance):
        i = 0
        achieved = True
        while (i < 6 and achieved) :
            if not(b[i]) :
                if abs(current[i] - desired[i]) > tolerance[i] : 
                    achieved = False
            i += 1
            
        if achieved :
            self.current_waypoint['achieved'] = True
            
        return achieved
    

    def legacyReq(self, req, now):
        """ A request is legacy if:
            - There is no requests
            - The request is the same than the latter
            - The requester is the same than the latter
            - The new request has more priority
            - 2 seconds has happen from the last request update
            - The last request is achieved """
            
        if not(self.current_waypoint['active']) : 
            #If there is the first request, save request information
            self.current_waypoint['active'] = True
            self.current_waypoint['requester'] = req.goal.requester
            self.current_waypoint['id'] = req.goal.id
            self.current_waypoint['priority'] = req.goal.priority
            self.current_waypoint['stamp_sec'] = now
            self.current_waypoint['achieved'] = False
            self.pid_x_z_yaw.reset(now)
            self.pid_x_y_z_yaw.reset(now)
            self.pid_relative_x_y_z_yaw.reset(now)
       
            return True
        elif (self.current_waypoint['requester'] == req.goal.requester and self.current_waypoint['id'] == req.goal.id) :
            #Same request
            self.current_waypoint['stamp_sec'] = now
            return True
        elif (self.current_waypoint['requester'] == req.goal.requester and self.current_waypoint['id'] != req.goal.id) :
            #Different request from the same requester
            self.current_waypoint['active'] = True
            self.current_waypoint['id'] = req.goal.id
            self.current_waypoint['priority'] = req.goal.priority
            self.current_waypoint['stamp_sec'] = now
            self.current_waypoint['achieved'] = False
            self.pid_x_z_yaw.reset(now)
            self.pid_x_y_z_yaw.reset(now)
            self.pid_relative_x_y_z_yaw.reset(now)
            return True
        elif (now > (self.current_waypoint['stamp_sec'] + 2.0)) or (req.goal.priority >  self.current_waypoint['priority']) or self.current_waypoint['achieved']:
            #Last petition is out-dated or
            #the new request has more priority or
            #the current waypoint has been achieved
            self.current_waypoint['active'] = True
            self.current_waypoint['requester'] = req.goal.requester
            self.current_waypoint['id'] = req.goal.id
            self.current_waypoint['priority'] = req.goal.priority
            self.current_waypoint['stamp_sec'] = now
            self.current_waypoint['achieved'] = False
            self.pid_x_z_yaw.reset(now)
            self.pid_x_y_z_yaw.reset(now)
            self.pid_relative_x_y_z_yaw.reset(now)
            return True
        else :
            print "ERROR! Pilot has a more priority request to serve"
            return False
    
    
    def moveMode_X_Z_YAW(self, req, now):
        x_error = 0.0
        z_error = 0.0
        yaw_error = 0.0
        tolerance = [req.position_tolerance.x, 
                     req.position_tolerance.y, 
                     req.position_tolerance.z, 
                     req.orientation_tolerance.roll, 
                     req.orientation_tolerance.pitch, 
                     req.orientation_tolerance.yaw]
        
        # Compute Error
        
        # Altitude mode
        if req.altitude_mode : 
            #TODO: Hem falta llegir la current altitude (hauria de ser ella enlloc de _p[2])!!!
            z_error = self.altitude - req.altitude 
            
            desired = [req.position.north, 
                       req.position.east, 
                       req.altitude, 
                       req.orientation.roll, 
                       req.orientation.pitch, 
                       req.orientation.yaw]
            
            current = [self.p[0], 
                       self.p[1], 
                       self.altitude, 
                       self.p[3], 
                       self.p[4], 
                       self.p[5]]
        
        # Depth Mode
        else : 
            z_error = req.position.depth - self.p[2]
            
            desired = [req.position.north, 
                       req.position.east, 
                       req.position.depth, 
                       req.orientation.roll, 
                       req.orientation.pitch, 
                       req.orientation.yaw]
            
            current = self.p
            
        inc_x = req.position.north - self.p[0]
        inc_y = req.position.east - self.p[1]
        desired_yaw = math.atan2(inc_y, inc_x)
        yaw_error = cola2_lib.normalizeAngle(desired_yaw - self.p[5])
        
        if abs(yaw_error) < self.max_angle_error :
            x_error = math.sqrt(pow(inc_x,2) + pow(inc_y,2))
       
        error = array([x_error, 0.0, z_error, 0.0, 0.0, yaw_error])
        
        #Compute Body Velocity request
        bvr = self.pid_x_z_yaw.compute(error, zeros(6), now)
        
        # Compute feedback
        success = self.checkTolerance([False, False, req.disable_axis.z, True, True, True], 
                                      current, 
                                      desired, 
                                      tolerance)

        # When moving with waypoint mode (normaly short distances) reduce the surge velocity
        max = list(self.max_velocity)
        max[0] = max[0] / 2.0

        return [success, bvr * max]
    

    def moveMode_LOS(self, previous_req, req, now):
        z_error = 0.0
        yaw_error = 0.0
        tolerance = [req.position_tolerance.x, 
                     req.position_tolerance.y, 
                     req.position_tolerance.z, 
                     req.orientation_tolerance.roll, 
                     req.orientation_tolerance.pitch, 
                     req.orientation_tolerance.yaw]
        
        # Compute Error
        
        # Altitude mode
        if req.altitude_mode : 
            #TODO: Hem falta llegir la current altitude (hauria de ser ella enlloc de _p[2])!!!
            z_error = self.altitude - req.altitude 
            
            desired = [req.position.north, 
                       req.position.east, 
                       req.altitude, 
                       req.orientation.roll, 
                       req.orientation.pitch, 
                       req.orientation.yaw]
            
            current = [self.p[0], 
                       self.p[1], 
                       self.altitude, 
                       self.p[3], 
                       self.p[4], 
                       self.p[5]]
        
        # Depth Mode
        else : 
            z_error = req.position.depth - self.p[2]
            desired = [req.position.north, 
                       req.position.east, 
                       req.position.depth, 
                       req.orientation.roll, 
                       req.orientation.pitch, 
                       req.orientation.yaw]
            
            current = self.p
            
        #Compute cross-track error
        alpha = math.atan2(req.position.east - previous_req.position.east, req.position.north - previous_req.position.north)
        
        #print "Alpha: " + str(alpha)
        e = -(self.p[0] - previous_req.position.north)*sin(alpha) + (self.p[1] - previous_req.position.east)*cos(alpha)
        #print "e:" + str(e)
        
        beta = math.atan2(self.v[1], self.v[0])
        #print "beta: " + str(beta)
        
        delta = 4.0
        #print "cross track error: " + str(math.atan(-e/delta))
        
        yaw_error = self.p[5] - (alpha + math.atan(-e/delta)) + beta
        yaw_error = cola2_lib.normalizeAngle(yaw_error)
        #print "yaw_error: " + str(yaw_error)
        
        error = array([0.0, 0.0, z_error, 0.0, 0.0, -yaw_error])
        
        #Compute Body Velocity request
        bvr = self.pid_x_z_yaw.compute(error, zeros(6), now)
        bvr *= self.max_velocity
        
        #TODO: All these vars have to be defined in some other place
        #Set x velocity to a constant value that depends on the yaw_error
        min_yaw_error = 0.05 #6 degrees
        max_yaw_error = self.max_angle_error
        min_x_v = self.min_velocity_los[0]
        max_x_v = self.max_velocity[0]
        
        SURGE_DEPEND_ON_YAW = False
        
        if SURGE_DEPEND_ON_YAW:
            error_angle = atan2(max_x_v - min_x_v, max_yaw_error - min_yaw_error)
            if abs(yaw_error) < min_yaw_error: bvr[0] = max_x_v
            elif abs(yaw_error) > max_yaw_error: bvr[0] = min_x_v
            else: bvr[0] = min_x_v + tan(error_angle)*(max_yaw_error - abs(yaw_error))
        else:
            bvr[0] = max_x_v
           
        #When the vehicle reaches or leaves a way-point the forward velocity follow a trapezoid
        #from min_distance to 0.0 distance respect to the way-point, the forward velocity decreases linearly 
        #from 100% of the computed value to min_tant_per_one*100 % of it.
        
        #TODO: All these vars have to be defined in some other place
        min_tant_per_one = 0.10     #tant per one
        min_distance = 5.0          #meters, this value has to be bigger than the acceptance sphere
        dist_angle = atan2(1-min_tant_per_one, min_distance)
        
        #distance to current way-point
        distance_current = sqrt((req.position.north-self.p[0])**2 + (req.position.east-self.p[1])**2)
        
        #distance to previous way-point
        distace_previous = sqrt((previous_req.position.north-self.p[0])**2 + (previous_req.position.east-self.p[1])**2)
        
        #Take the smaller one
        distance = min(distance_current, distace_previous)
        
        if distance < min_distance:
            tant_per_one = min_tant_per_one + tan(dist_angle)*distance
            bvr[0] *= tant_per_one
            if bvr[0] < min_x_v: bvr[0] = min_x_v    
       
        # Compute feedback
        success = self.checkTolerance([False, False, req.disable_axis.z, True, True, True], 
                                      current, 
                                      desired, 
                                      tolerance)
        
        #Check if the limit line has been crossed. Where the limit line is the line perpendicular to 
        #the desired path that pass through the current way-point, 
        inc_y = req.position.east - previous_req.position.east
        if inc_y != 0.0 :
            m_l = -(req.position.north - previous_req.position.north) / inc_y
        else: 
            m_l = 999999.9  
        c_l = -m_l * req.position.north + req.position.east
        current_d = (m_l * self.p[0] - self.p[1] + c_l)/sqrt(m_l**2 + 1)
        signe = (m_l * previous_req.position.north - previous_req.position.east + c_l)/sqrt(m_l**2 + 1)
        if signe * current_d < 0.0: 
            # The vehicle has crossed the line
            if req.disable_axis.z:
            # If z is uncontrolled
                success = True
            elif abs(current[2] - desired[2]) < tolerance[2]: 
            # If Z is ok, success = True. 
                success = True
            else:
            # Otherwise X and Yaw = 0.0 and success = false
                bvr[0] = 0.0
                bvr[5] = 0.0
                success = False
            
        # print "distance: ", current_d
        # print "signe: ", signe
        
        return [success, bvr]
    

    def moveMode_X_Y_Z_YAW(self, req, now):
        tolerance = [req.position_tolerance.x, 
                     req.position_tolerance.y, 
                     req.position_tolerance.z, 
                     req.orientation_tolerance.roll, 
                     req.orientation_tolerance.pitch, 
                     req.orientation_tolerance.yaw]
        z_error = 0.0
        yaw_error = 0.0
        x_error = 0.0
        y_error = 0.0
    
        # Compute Errors
        # Altitude mode
        if req.altitude_mode : 
            z_error = self.altitude - req.altitude
            
            desired = [req.position.north, 
                       req.position.east, 
                       req.altitude, 
                       req.orientation.roll, 
                       req.orientation.pitch, 
                       req.orientation.yaw]
            
            current = [self.p[0], 
                       self.p[1], 
                       self.altitude, 
                       self.p[3], 
                       self.p[4], 
                       self.p[5]]
        # Depth Mode
        else : 
            z_error = req.position.depth - self.p[2]
            
            desired = [req.position.north, 
                       req.position.east, 
                       req.position.depth, 
                       req.orientation.roll, 
                       req.orientation.pitch, 
                       req.orientation.yaw]
            
            current = self.p
            
        yaw_error = cola2_lib.normalizeAngle(req.orientation.yaw - self.p[5])
        
        [x_error, y_error] = self.computePosError(self.p[0], 
                                                  self.p[1], 
                                                  self.p[5], 
                                                  req.position.north, 
                                                  req.position.east)
        
        error = array([x_error, y_error, z_error, 0.0, 0.0, yaw_error])
        
        #Compute Body Velocity request
        bvr = self.pid_x_y_z_yaw.compute(error, zeros(6), now)
        
        # Send Body Velocity Request
        if req.disable_axis.z == True:
            bvr[2] = 0.0
        if req.disable_axis.yaw == True:
            bvr[5] = 0.0
            
        success = self.checkTolerance([False, False, req.disable_axis.z, True, True, req.disable_axis.yaw], 
                                      current, 
                                      desired, 
                                      tolerance)
        print 'current: ', current
        print 'desired: ', desired
        print 'tolerance: ', tolerance
             
        return [success, bvr*self.max_velocity]
    
    
    def moveMode_Relative_X_Y_Z_YAW(self, req, now):
        x_error = req.position.north
        y_error = req.position.east
        z_error = req.position.depth
        yaw_error = req.orientation.yaw
        
        tolerance = [req.position_tolerance.x, 
                     req.position_tolerance.y, 
                     req.position_tolerance.z, 
                     req.orientation_tolerance.roll, 
                     req.orientation_tolerance.pitch, 
                     req.orientation_tolerance.yaw]
        
        desired = [req.position.north, 
                   req.position.east, 
                   req.position.depth, 
                   req.orientation.roll, 
                   req.orientation.pitch, 
                   req.orientation.yaw]
        
        current = zeros(6)
        error = zeros(6)
        
        # Move first the Z and Yaw DoF and then X and Y
        if yaw_error < self.max_angle_error :
            error = array([x_error, y_error, z_error, 0.0, 0.0, yaw_error])
        else :
            error = array([0.0, 0.0, z_error, 0.0, 0.0, yaw_error])
        
        # Compute Body Velocity request
        bvr = self.pid_relative_x_y_z_yaw.compute(error, zeros(6), now)
        
        # Send Body Velocity Request
        success = self.checkTolerance([False, False, req.disable_axis.z, True, True, req.disable_axis.yaw], 
                                      current, 
                                      desired, 
                                      tolerance)
        
        return [success, bvr*self.max_velocity]
    
    
    def checkAbsoluteNavigationMode(self, b):
        if(not(b.x) and b.y and b.roll and b.pitch and not(b.yaw)) :
            return MovementMode.ABSOLUTE_X_Z_YAW
        elif(not(b.x) and not(b.y) and b.roll and b.pitch) :
            return MovementMode.ABSOLUTE_X_Y_Z_YAW
        else :
            return MovementMode.ERROR
    
    
    def checkRelativeNavigationMode(self, b):
        if(not(b.x) and not(b.y) and b.roll and b.pitch) :
            return MovementMode.RELATIVE_X_Y_Z_YAW
        else :
            return MovementMode.ERROR
        
        
    def updateNav(self, p, v, altitude):        
        self.p = p
        self.v = v
        self.altitude = altitude
