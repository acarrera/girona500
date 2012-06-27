#!/usr/bin/env python

# ROS imports
import roslib 
roslib.load_manifest('navigation_g500')
import rospy
import tf
from tf.transformations import euler_from_quaternion
import tf_conversions.posemath as pm
import PyKDL
import math
import cola2_lib
from ekf_g500 import EKFG500
import threading

# Msgs imports
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose
from navigation_g500.msg import *
from sensor_msgs.msg import Imu
from auv_msgs.msg import NavSts
from safety_g500.msg import NavSensorsStatus
from std_srvs.srv import Empty, EmptyResponse
from navigation_g500.srv import SetNE, SetNEResponse, SetNERequest


# Python imports
from numpy import *

INVALID_ALTITUDE = -32665
SAVITZKY_GOLAY_COEFFS = [0.2,  0.1,  0. , -0.1, -0.2]
# Larger filter introduce to much delay
# SAVITZKY_GOLAY_COEFFS = [0.10714286, 0.07142857, 0.03571429, 0., -0.03571429, -0.07142857, -0.10714286]
# SAVITZKY_GOLAY_COEFFS = [-0.05827506,  0.05710956,  0.1033411 ,  0.09770785,  0.05749806, 0. , -0.05749806, -0.09770785, -0.1033411 , -0.05710956, 0.05827506]
       
class Navigator :

    
    def __init__(self, name):
        """ Merge different navigation sensor values  """
        self.name = name
        
        # To filter yaw rate
        self.heading_buffer = []
        self.savitzky_golay_coeffs = SAVITZKY_GOLAY_COEFFS
        self.lock = threading.Lock() # TODO: Should I use RLock instead? 
        
        # Create Odometry msg
        self.odom = Odometry()
        self.nav = NavSts()
        
        # self.listener = tf.TransformListener()
        self.getConfig()
        
        #Create static transformations
        self.dvl_tf = self.computeTf(self.dvl_tf_data)
        self.imu_tf = self.computeTf(self.imu_tf_data)        
        self.svs_tf = self.computeTf(self.svs_tf_data)
        
        #Initialize flags
        self.init = False
        self.imu_data = False
        self.gps_data = not self.gps_update
        self.init_north = 0.0
        self.init_east = 0.0
        self.last_prediction = rospy.Time.now()
        self.altitude = INVALID_ALTITUDE
        self.bottom_status = 0
        
        #init last sensor update
        self.init_time = rospy.Time.now()
        self.dvl_last_update = self.init_time
        self.imu_last_update = self.init_time
        self.svs_last_update = self.init_time
        self.dvl_init = False
        self.imu_init = False
        self.svs_init = False
        self.gps_init_samples_list = []
        
        rospy.Timer(rospy.Duration(self.check_sensors_period), self.checkSensors)

        #Init Kalman filter
        self.ekf = EKFG500(self.model_covariance, 
                           self.gps_position_covariance, 
                           self.dvl_bottom_velocity_covariance,
                           self.dvl_water_velocity_covariance)
        
        # Create publisher
        self.pub_odom = rospy.Publisher("/navigation_g500/odometry", Odometry)
        self.pub_nav_sts = rospy.Publisher("/navigation_g500/nav_sts", NavSts)
        self.pub_nav_sensors_status = rospy.Publisher("/safety_g500/nav_sensors_status", NavSensorsStatus)
        
        # Create Subscriber
        rospy.Subscriber("/navigation_g500/teledyne_explorer_dvl", TeledyneExplorerDvl, self.updateTeledyneExplorerDvl)
        rospy.Subscriber("/navigation_g500/valeport_sound_velocity", ValeportSoundVelocity, self.updateValeportSoundVelocity)
        rospy.Subscriber("/navigation_g500/imu", Imu, self.updateImu)
        if self.gps_update :
            rospy.Subscriber("/navigation_g500/fastrax_it_500_gps", FastraxIt500Gps, self.updateGps)

        #Create services
        self.reset_navigation = rospy.Service('/navigation_g500/reset_navigation', Empty, self.resetNavigation)
        self.reset_navigation = rospy.Service('/navigation_g500/set_navigation', SetNE, self.setNavigation)
   

    def resetNavigation(self, req):
        rospy.loginfo("%s: Reset Navigation", self.name)
        
        self.lock.acquire()   
        x = self.ekf.getStateVector()
        x[0] = 0.0
        x[1] = 0.0
        self.ekf.initialize(x)
        self.lock.release()
        
        return EmptyResponse()
        
        
    def setNavigation(self, req):
        rospy.loginfo("%s: Set Navigation to: \n%s", self.name, req)    
        
        self.lock.acquire()   
        x = self.ekf.getStateVector()
        x[0] = req.north
        x[1] = req.east
        self.ekf.initialize(x)
        self.lock.release()
        
        ret = SetNEResponse()
        ret.success = True
        return ret
    
        
    def checkSensors(self, event):
#        rospy.loginfo("Check nav sensors")
        if self.dvl_init and self.imu_init and self.svs_init and self.init:
            now = rospy.Time.now()
            nav_sensors_status = NavSensorsStatus()
            nav_sensors_status.header.stamp = now
            
            if (now - self.dvl_last_update).to_sec() > self.dvl_max_period_error:
    #            rospy.logerr("%s: DVL data error", self.name)
                nav_sensors_status.dvl_status = False
            else :
                nav_sensors_status.dvl_status = True
                
            if (now - self.imu_last_update).to_sec() > self.imu_max_period_error:
    #            rospy.logerr("%s: IMU data error", self.name)
                nav_sensors_status.imu_status = False
            else :
                nav_sensors_status.imu_status = True
                
            if (now - self.svs_last_update).to_sec() > self.svs_max_period_error:
    #            rospy.logerr("%s: SVS data error", self.name)
                nav_sensors_status.svs_status = False
            else :
                nav_sensors_status.svs_status = True
                
            self.pub_nav_sensors_status.publish(nav_sensors_status)
        else :
            if (rospy.Time.now() - self.init_time).to_sec() > self.max_init_time:
                rospy.logfatal("%s: Not all nav sensors have been initialized!", self.name)
                rospy.logfatal("%s: DVL init: %s", self.name, self.dvl_init)
                rospy.logfatal("%s: SVS init: %s", self.name, self.svs_init)
                rospy.logfatal("%s: IMU init: %s", self.name, self.imu_init)
                rospy.logfatal("%s: EKF filter init: %s", self.name, self.init)
                
          
    def updateGps(self, gps):
        if gps.data_quality >= 1 and gps.latitude_hemisphere >= 0 and gps.longitude_hemisphere >= 0:
            if not self.gps_data :
                self.gps_init_samples_list.append([gps.north, gps.east])
                if len(self.gps_init_samples_list) >= self.gps_init_samples:
                    self.gps_data = True
                    [self.init_north, self.init_east] = median(array(self.gps_init_samples_list), axis=0)
                    rospy.loginfo('%s, GPS init data: %sN, %sE', self.name, self.init_north, self.init_east)
            else:
                distance = sqrt((self.nav.position.north - gps.north)**2 + 
                                (self.nav.position.east - gps.east)**2)
                #rospy.loginfo("%s, Distance: %s", self.name, distance)
                
                #TODO: Roght now the GPS is only used to initialize the navigation not for updating it!!!
                if distance < 0.1:
                    # If the error between the filter and the GPS is small, update the kalman filter
                    self.odom.pose.pose.position.x = gps.north
                    self.odom.pose.pose.position.y = gps.east
                    
                    # Update EKF
                    if self.makePrediction():
                        z = array([gps.north, gps.east])
                        
                        self.lock.acquire()
                        self.ekf.gpsUpdate(z)
                        self.lock.release()
                        
                        self.publishData()
                        
                elif distance < 50.0:
                    #If the error is big, reinitialize the navigation filter if GPS is very far
                    #self.init = False
                    #self.gps_data = False
                    #rospy.loginfo("%s, Filter need to be reinitialized", self.name)
                    distance = 0
                else:
                    #If the error is huge, we assume that the GPS data is not valid
                    #rospy.loginfo("%s, Invalid GPS data received:\n", self.name)
                    distance = 0
        
        
    def updateTeledyneExplorerDvl(self, dvl):
        self.dvl_last_update = rospy.Time.now()
        self.dvl_init = True
        
        # If dvl_update == 0 --> No update
        # If dvl_update == 1 --> Update wrt bottom
        # If dvl_update == 2 --> Update wrt water
        dvl_update = 0
        
        if dvl.bi_status == "A" and dvl.bi_error > -32.0:
            if abs(dvl.bi_x_axis) < self.dvl_max_v and abs(dvl.bi_y_axis) < self.dvl_max_v and abs(dvl.bi_z_axis) < self.dvl_max_v : 
                v = PyKDL.Vector(dvl.bi_x_axis, dvl.bi_y_axis, dvl.bi_z_axis)
                dvl_update = 1
        elif dvl.wi_status == "A" and dvl.wi_error > -32.0:
            if abs(dvl.wi_x_axis) < self.dvl_max_v and abs(dvl.wi_y_axis) < self.dvl_max_v and abs(dvl.wi_z_axis) < self.dvl_max_v : 
                v = PyKDL.Vector(dvl.wi_x_axis, dvl.wi_y_axis, dvl.wi_z_axis)
                dvl_update = 2
        
        #Filter to check if the altitude is reliable
        if dvl.bi_status == "A" and dvl.bi_error > -32.0:
            self.bottom_status =  self.bottom_status + 1
        else:
            self.bottom_status = 0
        
        if self.bottom_status > 4:
            self.altitude = dvl.bd_range
        else:
            self.altitude = INVALID_ALTITUDE
            
        if dvl_update != 0:
            #Rotate DVL velocities and Publish
            #Compte! EL DVL no es dextrogir i s'ha de negar la Y
            vr = self.dvl_tf.M * v
            vr2 = array([vr[0], -vr[1], vr[2]])
            
            #Ara ja tenim la velocitat lineal en el DVL representada en eixos de vehicle
            #falta calcular la velocitat lineal al centre del vehicle en eixos del vehicle
            angular_velocity = array([self.odom.twist.twist.angular.x,
                                      self.odom.twist.twist.angular.y,
                                      self.odom.twist.twist.angular.z])
            distance = self.dvl_tf_data[0:3]
            vr2 = vr2 - cross(angular_velocity, distance)
            
            #Copy data to odometry message
            self.odom.twist.twist.linear.x = vr2[0]
            self.odom.twist.twist.linear.y = vr2[1]
            self.odom.twist.twist.linear.z = vr2[2]
            
            # Update EKF
            if self.makePrediction():
                z = array([vr2[0], vr2[1], vr2[2]])
        
                self.lock.acquire()
                if dvl_update == 1: 
                    self.ekf.dvlUpdate(z, 'bottom')
                else:
                    self.ekf.dvlUpdate(z, 'water')
                self.lock.release()
        
                self.publishData()
        else:
            # rospy.loginfo('%s, invalid DVL velocity measurement!', self.name)
            pass
       
        
    
    def updateValeportSoundVelocity(self, svs):
        self.svs_last_update = rospy.Time.now()
        self.svs_init = True
        
        svs_data = PyKDL.Vector(.0, .0, svs.pressure)
        angle = tf.transformations.euler_from_quaternion([self.odom.pose.pose.orientation.x,
                                                          self.odom.pose.pose.orientation.y,
                                                          self.odom.pose.pose.orientation.z,
                                                          self.odom.pose.pose.orientation.w])
        vehicle_rpy = PyKDL.Rotation.RPY(angle[0], angle[1], angle[2])
        svs_trans = self.svs_tf.p
        svs_trans = vehicle_rpy * svs_trans
        svs_data = svs_data - svs_trans
        self.odom.pose.pose.position.z = svs_data[2] 


    def updateImu(self, imu):
        self.imu_last_update = rospy.Time.now()
        self.imu_init = True
        
        if not self.imu_data :
            angle = euler_from_quaternion([imu.orientation.x, imu.orientation.y, imu.orientation.z, imu.orientation.w])
            imu_data =  PyKDL.Rotation.RPY(angle[0], angle[1], angle[2])
            imu_data = imu_data * self.imu_tf.M
            angle = imu_data.GetRPY()   
            self.last_imu_orientation = angle
            self.last_imu_update = imu.header.stamp
            
            # Initialize heading buffer in order to apply a savitzky_golay derivation
            if len(self.heading_buffer) == 0:
                self.heading_buffer.append(angle[2])
                
            inc = cola2_lib.normalizeAngle(angle[2] - self.heading_buffer[-1])
            self.heading_buffer.append(self.heading_buffer[-1] + inc)
            
            if len(self.heading_buffer) == len(self.savitzky_golay_coeffs):
                self.imu_data = True
        else:
            angle = euler_from_quaternion([imu.orientation.x, imu.orientation.y, imu.orientation.z, imu.orientation.w])
            imu_data =  PyKDL.Rotation.RPY(angle[0], angle[1], angle[2])
            imu_data = imu_data * self.imu_tf.M
            angle = imu_data.GetRPY()   
            angle_quaternion = tf.transformations.quaternion_from_euler(angle[0], angle[1], angle[2])
            self.odom.pose.pose.orientation.x = angle_quaternion[0]
            self.odom.pose.pose.orientation.y = angle_quaternion[1]
            self.odom.pose.pose.orientation.z = angle_quaternion[2]
            self.odom.pose.pose.orientation.w = angle_quaternion[3]
            
            # Derive angular velocities from orientations ####################### 
            period = (imu.header.stamp - self.last_imu_update).to_sec()
            
            # For yaw rate we apply a savitzky_golay derivation
            inc = cola2_lib.normalizeAngle(angle[2] - self.heading_buffer[-1])
            self.heading_buffer.append(self.heading_buffer[-1] + inc)
            self.heading_buffer.pop(0)
            self.odom.twist.twist.angular.z = convolve(self.heading_buffer, self.savitzky_golay_coeffs, mode='valid') / period

            # TODO: Roll rate and Pitch rate should be also savitzky_golay derivations
            self.odom.twist.twist.angular.x = cola2_lib.normalizeAngle(angle[0] - self.last_imu_orientation[0]) / period 
            self.odom.twist.twist.angular.y = cola2_lib.normalizeAngle(angle[1] - self.last_imu_orientation[1]) / period 

            self.last_imu_orientation = angle
            self.last_imu_update = imu.header.stamp          
            #####################################################################
            
            if self.makePrediction():
                self.lock.acquire()
                self.ekf.updatePrediction()
                self.lock.release()
        
                self.publishData()

        
    def makePrediction(self):
        if not self.init :
            if self.imu_data and self.gps_data:
                self.last_prediction = rospy.Time.now()
        
                self.lock.acquire()
                self.ekf.initialize(array([self.init_north, self.init_east, 0.0, 0.0, 0.0]))
                self.lock.release()
        
                self.init = True
            return False
        else :
            angle = euler_from_quaternion([self.odom.pose.pose.orientation.x,
                                           self.odom.pose.pose.orientation.y,
                                           self.odom.pose.pose.orientation.z,
                                           self.odom.pose.pose.orientation.w])
            now = rospy.Time.now()
            t = (now - self.last_prediction).to_sec()
            self.last_prediction = now
        
            self.lock.acquire()
            self.ekf.prediction(angle, t)
            self.lock.release()
        
            return True
        
            
    def publishData(self):
        if self.init:
            self.lock.acquire()
            x = self.ekf.getStateVector()
            self.lock.release()
        
            angle = euler_from_quaternion([self.odom.pose.pose.orientation.x,
                                           self.odom.pose.pose.orientation.y,
                                           self.odom.pose.pose.orientation.z,
                                           self.odom.pose.pose.orientation.w])            
            
        #   Create header    
            self.odom.header.stamp = rospy.Time.now()
            self.odom.header.frame_id = "girona500"
            self.odom.child_frame_id = "world"
                       
            #Fil Nav status topic
            self.nav.header = self.odom.header
            self.nav.position.north = x[0]
            self.nav.position.east = x[1]
            self.nav.position.depth = self.odom.pose.pose.position.z
            self.nav.body_velocity.x = x[2]
            self.nav.body_velocity.y = x[3]
            self.nav.body_velocity.z = x[4]
            self.nav.orientation.roll = angle[0]
            self.nav.orientation.pitch = angle[1]
            self.nav.orientation.yaw = angle[2]
            self.nav.orientation_rate.roll = self.odom.twist.twist.angular.x
            self.nav.orientation_rate.pitch = self.odom.twist.twist.angular.y
            self.nav.orientation_rate.yaw = self.odom.twist.twist.angular.z
            self.nav.altitude = self.altitude
            
            #Publish topics
            self.pub_nav_sts.publish(self.nav)
            self.pub_odom.publish(self.odom)
            
            #Publish TF
            br = tf.TransformBroadcaster()
            br.sendTransform((self.nav.position.north, self.nav.position.east, self.nav.position.depth),                   
                             tf.transformations.quaternion_from_euler(self.nav.orientation.roll, self.nav.orientation.pitch, self.nav.orientation.yaw),
                             self.odom.header.stamp,
                             self.odom.header.frame_id,
                             self.odom.child_frame_id)
        else :
            self.last_time = rospy.Time.now()
            self.init = True
    
    
    def computeTf(self, tf):
        r = PyKDL.Rotation.RPY(math.radians(tf[3]), math.radians(tf[4]), math.radians(tf[5]))
#        rospy.loginfo("Rotation: %s", str(r))
        v = PyKDL.Vector(tf[0], tf[1], tf[2])
#        rospy.loginfo("Vector: %s", str(v))
        frame = PyKDL.Frame(r, v)
#        rospy.loginfo("Frame: %s", str(frame))
        return frame
        
        
    def getConfig(self):
        if rospy.has_param("teledyne_explorer_dvl/tf") :
            self.dvl_tf_data = array(rospy.get_param("teledyne_explorer_dvl/tf"))
        else:
            rospy.logfatal("teledyne_explorer_dvl/tf param not found")

        if rospy.has_param("tritech_igc_gyro/tf") :
            self.imu_tf_data = array(rospy.get_param("tritech_igc_gyro/tf"))
        else:
            rospy.logfatal("tritech_igc_gyro/tf param not found")

        if rospy.has_param("valeport_sound_velocity/tf") :
            self.svs_tf_data = array(rospy.get_param("valeport_sound_velocity/tf"))
        else:
            rospy.logfatal("valeport_sound_velocity/tf param not found")
            
#       Sensors & model covariance
        if rospy.has_param("navigator/dvl_bottom_covariance") :
            self.dvl_bottom_velocity_covariance = array(rospy.get_param('navigator/dvl_bottom_covariance'))
        else:
            rospy.logfatal("navigator/dvl_bottom_covariance param not found")
        
        if rospy.has_param("navigator/dvl_water_covariance") :
            self.dvl_water_velocity_covariance = array(rospy.get_param('navigator/dvl_water_covariance'))
        else:
            rospy.logfatal("navigator/dvl_water_covariance param not found")

        if rospy.has_param("navigator/gps_covariance") :
            self.gps_position_covariance = rospy.get_param('navigator/gps_covariance')
        else:
            rospy.logfatal("navigator/gps_covariance param not found")
            
        if rospy.has_param("navigator/model_covariance") :
            self.model_covariance = rospy.get_param('navigator/model_covariance')
        else:
            rospy.logfatal("navigator/model_covariance param not found")
        
        if rospy.has_param("navigator/dvl_max_v") :
            self.dvl_max_v = rospy.get_param('navigator/dvl_max_v')
        else:
            rospy.logfatal("navigator/dvl_max_v not found")
            
        if rospy.has_param("navigator/gps_update"):
            self.gps_update = rospy.get_param('navigator/gps_update')
        else:
            rospy.logfatal("navigator/gps_update not found")
        
        if rospy.has_param("navigator/gps_init_samples"):
            self.gps_init_samples = rospy.get_param('navigator/gps_init_samples')
        else:
            rospy.logfatal("navigator/gps_init_samples not found")
            
        if rospy.has_param("navigator/check_sensors_period"):
            self.check_sensors_period = rospy.get_param('navigator/check_sensors_period')
        else:
            rospy.logfatal("navigator/check_sensors_period not found")
            
        if rospy.has_param("navigator/dvl_max_period_error"):
            self.dvl_max_period_error = rospy.get_param('navigator/dvl_max_period_error')
        else:
            rospy.logfatal("navigator/dvl_max_period_error not found")

        if rospy.has_param("navigator/svs_max_period_error"):
            self.svs_max_period_error = rospy.get_param('navigator/svs_max_period_error')
        else:
            rospy.logfatal("navigator/csvs_max_period_error not found")
            
        if rospy.has_param("navigator/imu_max_period_error"):
            self.imu_max_period_error = rospy.get_param('navigator/imu_max_period_error')
        else:
            rospy.logfatal("navigator/imu_max_period_error not found")
            
        if rospy.has_param("navigator/max_init_time") :
            self.max_init_time = rospy.get_param("navigator/max_init_time")
        else:
            rospy.logfatal("navigator/max_init_time not found in param list")
            
if __name__ == '__main__':
    try:
        #   Init node
        rospy.init_node('navigator')
        navigator = Navigator(rospy.get_name())
        rospy.spin()
    except rospy.ROSInterruptException: pass
