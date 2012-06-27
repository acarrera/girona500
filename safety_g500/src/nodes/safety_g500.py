#!/usr/bin/env python

# ROS imports
import roslib 
roslib.load_manifest('safety_g500')
import rospy

from safety_g500.srv import *
from safety_g500.msg import *
from std_srvs.srv import Empty, EmptyResponse, EmptyRequest
import std_msgs.msg
from auv_msgs.msg import BodyVelocityReq
from auv_msgs.msg import NavSts
from threading import Thread
from safety_g500.srv import MissionTimeout
from std_msgs.msg import Int16
from auv_msgs.msg import GoalDescriptor
import cola2_lib

class ErrorLevel:
    OK = 0
    INFORMATIVE = 1
    ASK_AUTHORIZATION = 2
    ABORT_MISSION = 3
    CONTROLLED_SURFACE = 4
    SURFACE = 5
    EMERGENCY_SURFACE = 6
    DISABLE_PAYLOAD = 7
    

class ErrorCode:
    INIT = 15 
    DVL = 14
    IMU = 13
    SVS = 12
    NAV_STS = 11
    BAT_STS = 10
    LOW_BAT = 9
    INTERNAL_SENSORS_STS = 8
    WATER = 7
    TEMPERATURE = 6
    PRESSURE = 5
    HUMIDITY = 4
    INIT_FAIL = 3
    DVL_BOTTOM_FAIL = 2
    # TODO: WARNING!! int16 --> -32768 to 32768, but error code has been defined always positive
    # then ErrorCode 0 can't be used (it has to be always 0) 
    # oterwise we reach value 65535 that can not be transmitted using an int16.
    
    
class SafetyG500:
    def __init__(self, name):
        self.name = name
        self.getConfig()
        
        # to avoid multiple surface threads
        self.is_surfacing = False
        self.controlled_surface_enabled = False
        
        # Error level
        self.error_code = ['0' for i in range(16)]
        
        # Init navigation
        self.navigation = NavSts()
        self.navigation.position.depth = 0.0
        
        # Create Subscriber
        rospy.Subscriber("/safety_g500/internal_sensors", InternalSensors, self.updateInternalSensors)
        rospy.Subscriber("/safety_g500/nav_sensors_status", NavSensorsStatus, self.updateNavSensorsStatus)
        rospy.Subscriber("/safety_g500/ocean_server_batteries", BatteryLevel, self.updateBatteryLevel)
        rospy.Subscriber("/navigation_g500/nav_sts", NavSts, self.updateNavSts)
        
        # Create publisher
        self.pub_heart_beat = rospy.Publisher("/safety_g500/heart_beat", std_msgs.msg.Empty)
        self.pub_body_velocity_req = rospy.Publisher("/control_g500/body_velocity_req", BodyVelocityReq)
        self.pub_error_code = rospy.Publisher("/safety_g500/error_code", Int16)
        
        # Init sensor status
        self.internal_sensors_init = False
        self.nav_sensors_init = False
        self.batteries_init = False
        self.velocity_controller_enabled = False
        
        # Init sensor messages
        self.nav_sensors_status = NavSensorsStatus()
        self.internal_sensors = InternalSensors()
        self.battery_level = BatteryLevel()
        
        # Init time
        self.init_time = rospy.Time.now()
        self.nav_sensors_status.header.stamp = self.init_time
        self.internal_sensors.header.stamp = self.init_time
        self.battery_level.header.stamp = self.init_time
        
        # Init periodic check timer 
        rospy.Timer(rospy.Duration(self.periodic_check), self.periodicCheck)
        rospy.Timer(rospy.Duration(self.heart_beat_period), self.heartBeat)
        rospy.Timer(rospy.Duration(2.0), self.errorCode)
        
        #Create services
        self.controlled_surface_service = rospy.Service('/safety_g500/controlled_surface', Empty, self.controlledSurfaceSrv)
        self.surface_service = rospy.Service('/safety_g500/surface', Empty, self.surfaceSrv)
        self.emergency_surface_service = rospy.Service('/safety_g500/emergency_surface', Empty, self.emergencySurfaceSrv)        
        self.disable_surface_service = rospy.Service('/safety_g500/disable_surface', Empty, self.disableSurfaceSrv)        
        self.reset_ms_service = rospy.Service('/safety_g500/reset_ms', Empty, self.resetMSSrv)        
  
        # Wait for necessary services
        # TODO: posar timeouts a tots els wait_for_service !!!
        try:
            rospy.wait_for_service('/control_g500/enable_velocity_controller', 5)
            rospy.wait_for_service('/control_g500/disable_velocity_controller', 5)
        
            self.enable_velocity_controller_srv = rospy.ServiceProxy('/control_g500/enable_velocity_controller', Empty)
            self.disable_velocity_controller_srv = rospy.ServiceProxy('/control_g500/disable_velocity_controller', Empty)
        except rospy.exceptions.ROSException:
            rospy.logerr('%s, Error connecting to velocity Controller.', self.name)
        
        try: 
            rospy.wait_for_service('/mission_timeout', 5)
            # TODO: solve wait for android services
            rospy.sleep(1.0)
            self.mission_timeout_srv = rospy.ServiceProxy('/mission_timeout', MissionTimeout)
            mt = MissionTimeoutRequest()
            mt.start_mission = True
            mt.time_out = self.mission_time_out
            rospy.loginfo('call mission_timeout with: %s', mt)
            response = self.mission_timeout_srv(mt)
            if not response.success:
                rospy.logerr('%s, Error initializing Mission Timeout', self.name)
        except rospy.exceptions.ROSException:
            rospy.logerr('%s, Error initializing Mission Timeout. Are you in simulation mode?', self.name)
        
        # Enable set zero velocity behavior
        t = Thread(target=self.setZeroVelocity, args=())
        t.start()
        
        # Define stop function
        rospy.on_shutdown(self.stop)
    
    
    def stop(self):
        # Send mission end
        mt = MissionTimeoutRequest()
        mt.start_mission = False
        mt.time_out = 0
        rospy.loginfo('%s, Disable mission timeout: %s', self.name, mt)
        self.mission_timeout_srv(mt)
    
        
    def resetMSSrv(self, req):
        try:
            rospy.wait_for_service('digital_output', 5)
            digital_out_service = rospy.ServiceProxy('digital_output', DigitalOutput)
            digital_output = DigitalOutputRequest()
            digital_output.digital_out = 16
            digital_output.value = True
            digital_out_service(digital_output)
        except rospy.exceptions.ROSException:
            rospy.logerr('%s, Error reseting Mass Spectrometer.', self.name)
        
        return EmptyResponse()
    
    
    def controlledSurfaceSrv(self, req):
        self.controlledSurface()
        return EmptyResponse()
    
                 
    def surfaceSrv(self, req):
        self.surface()
        return EmptyResponse()
    
    
    def emergencySurfaceSrv(self, req):
        self.emergencySurface()
        return EmptyResponse()
    
    
    def disableSurfaceSrv(self, req):
        self.is_surfacing = False
        self.controlled_surface_enabled = False
        return EmptyResponse()
    
    
    def getConfig(self):
        if rospy.has_param("safety_g500/set_zero_velocity_depth") :
            self.set_zero_velocity_depth = rospy.get_param("safety_g500/set_zero_velocity_depth")
        else:
            rospy.logfatal("safety_g500/set_zero_velocity_depth")
        
        if rospy.has_param("safety_g500/mission_time_out") :
            self.mission_time_out = rospy.get_param("safety_g500/mission_time_out")
        else:
            rospy.logfatal("safety_g500/mission_time_out")
        
        if rospy.has_param("safety_g500/max_depth") :
            self.max_depth = rospy.get_param("safety_g500/max_depth")
        else:
            rospy.logfatal("safety_g500/max_depth not found in param list")
    
        if rospy.has_param("safety_g500/min_altitude") :
            self.min_altitude = rospy.get_param("safety_g500/min_altitude")
        else:
            rospy.logfatal("safety_g500/min_altitude not found in param list")
    
        if rospy.has_param("safety_g500/heart_beat_period") :
            self.heart_beat_period = rospy.get_param("safety_g500/heart_beat_period")
        else:
            rospy.logfatal("safety_g500/heart_beat_period not found in param list")
        
        if rospy.has_param("safety_g500/periodic_check") :
            self.periodic_check = rospy.get_param("safety_g500/periodic_check")
        else:
            rospy.logfatal("safety_g500/periodic_check not found in param list")
            
        if rospy.has_param("safety_g500/max_humidity") :
            self.max_humidity = rospy.get_param("safety_g500/max_humidity")
        else:
            rospy.logfatal("safety_g500/max_humidity not found in param list")
            
        if rospy.has_param("safety_g500/max_pressure") :
            self.max_pressure = rospy.get_param("safety_g500/max_pressure")
        else:
            rospy.logfatal("safety_g500/max_pressure not found in param list")
    
        if rospy.has_param("safety_g500/max_temperature") :
            self.max_temperature = rospy.get_param("safety_g500/max_temperature")
        else:
            rospy.logfatal("safety_g500/max_temperature not found in param list")

        if rospy.has_param("safety_g500/min_battery_level") :
            self.min_battery_level = rospy.get_param("safety_g500/min_battery_level")
        else:
            rospy.logfatal("safety_g500/min_battery_level not found in param list")
   
        if rospy.has_param("safety_g500/max_init_time") :
            self.max_init_time = rospy.get_param("safety_g500/max_init_time")
        else:
            rospy.logfatal("safety_g500/max_init_time not found in param list")

        if rospy.has_param("safety_g500/max_nav_sensors_period") :
            self.max_nav_sensors_period = rospy.get_param("safety_g500/max_nav_sensors_period")
        else:
            rospy.logfatal("safety_g500/max_nav_sensors_period not found in param list")
    
        if rospy.has_param("safety_g500/max_internal_sensors_period") :
            self.max_internal_sensors_period = rospy.get_param("safety_g500/max_internal_sensors_period")
        else:
            rospy.logfatal("safety_g500/max_internal_sensors_period not found in param list")
            
        if rospy.has_param("safety_g500/max_battery_level_period") :
            self.max_battery_level_period = rospy.get_param("safety_g500/max_battery_level_period")
        else:
            rospy.logfatal("safety_g500/max_battery_level_period not found in param list")
    
        if rospy.has_param("safety_g500/start_vc_once_init") :
            self.start_vc_once_init = rospy.get_param("safety_g500/start_vc_once_init")
        else:
            rospy.logfatal("safety_g500/start_vc_once_init not found in param list")

        if rospy.has_param("safety_g500/controlled_surface_depth") :
            self.controlled_surface_depth = rospy.get_param("safety_g500/controlled_surface_depth")
        else:
            rospy.logfatal("safety_g500/controlled_surface_depth not found in param list")


    def heartBeat(self, event):
        self.pub_heart_beat.publish(std_msgs.msg.Empty())
        
        
    def errorCode(self, event):
        error_code_str = ''.join(self.error_code)
        error_code_int16 = int(error_code_str, 2)
        self.pub_error_code.publish(std_msgs.msg.Int16(error_code_int16))
        
        
    def periodicCheck(self, event):
        if self.internal_sensors_init and self.batteries_init and self.nav_sensors_init:
            if not self.velocity_controller_enabled:
                if self.start_vc_once_init:
                    self.enable_velocity_controller_srv(EmptyRequest())
                self.velocity_controller_enabled = True
                rospy.loginfo("  VEHICLE INITIALIZED  ")
                self.error_code[ErrorCode.INIT] = '1'
                self.error_code[ErrorCode.INIT_FAIL] = '0'
                
            # Check Navigation sensors status has been received
            if (rospy.Time.now() - self.nav_sensors_status.header.stamp).to_sec() > self.max_nav_sensors_period:
                rospy.logerr("%s: Navigation sensors status message not received! Localization node probably not working.", self.name)
                self.error_code[ErrorCode.NAV_STS] = '1'
                self.recoveryAction(ErrorLevel.INFORMATIVE)
            else:
                self.error_code[ErrorCode.NAV_STS] = '0'
                
                
            if (rospy.Time.now() - self.internal_sensors.header.stamp).to_sec() > self.max_internal_sensors_period:
                rospy.logerr("%s: Internal sensors status message not received! Main board node probably not working.", self.name)
                self.error_code[ErrorCode.INTERNAL_SENSORS_STS] = '1'
                self.recoveryAction(ErrorLevel.INFORMATIVE)
            else:
                self.error_code[ErrorCode.INTERNAL_SENSORS_STS] = '0'
                
            if (rospy.Time.now() - self.battery_level.header.stamp).to_sec() > self.max_battery_level_period:
                rospy.logerr("%s: Battery level message not received! Ocean server driver probably not working.", self.name)
                self.error_code[ErrorCode.BAT_STS] = '1'
                self.recoveryAction(ErrorLevel.INFORMATIVE)
            else:
                self.error_code[ErrorCode.BAT_STS] = '0'
        else:
            if (rospy.Time.now() - self.init_time).to_sec() > self.max_init_time:
                rospy.logfatal("%s: Not all safety measures have been initialized!", self.name)
                self.error_code[ErrorCode.INIT_FAIL] = '1'
                self.recoveryAction(ErrorLevel.INFORMATIVE)


    def updateInternalSensors(self, sensors):
        self.internal_sensors = sensors
        #Add time stamp because arduino board has no time
        self.internal_sensors.header.stamp = rospy.Time.now()
        self.internal_sensors_init = True
        
        if sensors.humidity > self.max_humidity:
            rospy.logerr("%s: Humidity detected in vessel %s", self.name, sensors.id)
            self.error_code[ErrorCode.HUMIDITY] = '1'
            self.recoveryAction(ErrorLevel.INFORMATIVE)
        else:
            self.error_code[ErrorCode.HUMIDITY] = '0'
            
        if sensors.pressure > self.max_pressure:
            rospy.logerr("%s: Pressure detected in vessel %s", self.name, sensors.id)
            self.error_code[ErrorCode.PRESSURE] = '1'
            self.recoveryAction(ErrorLevel.INFORMATIVE)
        else:
            self.error_code[ErrorCode.PRESSURE] = '0'
            
        if sensors.temperature > self.max_temperature:
            rospy.logerr("%s: Temperature detected in vessel %s", self.name, sensors.id)
            self.error_code[ErrorCode.TEMPERATURE] = '1'
            self.recoveryAction(ErrorLevel.INFORMATIVE)
        else:
            self.error_code[ErrorCode.TEMPERATURE] = '0'
            
        if sensors.water_detected:
            rospy.logerr("%s: Water detected in vessel %s", self.name, sensors.id)
            self.error_code[ErrorCode.WATER] = '1'
            self.recoveryAction(ErrorLevel.INFORMATIVE)
        else:
            self.error_code[ErrorCode.WATER] = '0'
            
        
    def updateNavSensorsStatus(self, sensors):
        self.nav_sensors_status = sensors
        self.nav_sensors_init = True
        
        if not sensors.dvl_status:
            rospy.logerr("%s: DVL sensor is not publishing data", self.name)
            self.error_code[ErrorCode.DVL] = '1'
            self.recoveryAction(ErrorLevel.INFORMATIVE)
        else:
            self.error_code[ErrorCode.DVL] = '0'
            
        if not sensors.imu_status:
            rospy.logerr("%s: IMU sensor is not publishing data", self.name)
            self.error_code[ErrorCode.IMU] = '1'
            self.recoveryAction(ErrorLevel.INFORMATIVE)
        else:
            self.error_code[ErrorCode.IMU] = '0'
            
        if not sensors.svs_status:
            rospy.logerr("%s: SVS sensor is not publishing data", self.name)
            self.error_code[ErrorCode.SVS] = '1'
            self.recoveryAction(ErrorLevel.INFORMATIVE)
        else:
            self.error_code[ErrorCode.SVS] = '0'
    
    
    def updateBatteryLevel(self, battery):
        self.battery_level = battery
        self.batteries_init = True
        
        if battery.charge < (self.min_battery_level/2):
            rospy.logerr("%s: Battery below half the minimum charge (%s). Emergency surface enabled.", self.name, battery.charge)
            self.error_code[ErrorCode.LOW_BAT] = '1'
            self.recoveryAction(ErrorLevel.CONTROLLED_SURFACE)
        elif battery.charge < self.min_battery_level:
            rospy.logerr("%s: Battery below minimum charge (%s)", self.name, battery.charge)
            self.error_code[ErrorCode.LOW_BAT] = '1'
            self.recoveryAction(ErrorLevel.INFORMATIVE)
        else:
            self.error_code[ErrorCode.LOW_BAT] = '0'    
    
    
    def updateNavSts(self, nav):
        self.navigation = nav
        if  (nav.altitude > 0 and nav.altitude < self.min_altitude) or (nav.position.depth > self.max_depth):
            bvr = BodyVelocityReq()
            bvr.twist.linear.x = 0.0
            bvr.twist.linear.y = 0.0
            bvr.twist.linear.z = -0.5
            bvr.twist.angular.x = 0.0
            bvr.twist.angular.y = 0.0
            bvr.twist.angular.z = 0.0
            bvr.disable_axis.x = False
            bvr.disable_axis.y = False
            bvr.disable_axis.z = False
            bvr.disable_axis.roll = True
            bvr.disable_axis.pitch = True
            bvr.disable_axis.yaw = False
            bvr.goal.priority =  GoalDescriptor.PRIORITY_EMERGENCY
            bvr.goal.requester = self.name
            bvr.header.stamp = rospy.Time.now()
            self.pub_body_velocity_req.publish(bvr)
        
        if nav.altitude < 0:
            self.error_code[ErrorCode.DVL_BOTTOM_FAIL] = '1'
            # self.recoveryAction(ErrorLevel.INFORMATIVE)            
        else:
            self.error_code[ErrorCode.DVL_BOTTOM_FAIL] = '0'
            
    
    def recoveryAction(self, error):
        if error == ErrorLevel.INFORMATIVE:
            rospy.loginfo("%s: recovery action %s: INFORMATIVE", self.name, error)
        elif error == ErrorLevel.ASK_AUTHORIZATION:
            # TODO: Unused!!!
            rospy.loginfo("%s: recovery action %s: ASK_AUTHORIZATION", self.name, error)
        elif error == ErrorLevel.ABORT_MISSION:
            self.abortMission()
        elif error == ErrorLevel.CONTROLLED_SURFACE:
            self.controlledSurface() 
        elif error == ErrorLevel.SURFACE:
            self.surface()
        elif error == ErrorLevel.EMERGENCY_SURFACE:
            self.emergencySurface()
        elif error == ErrorLevel.DISABLE_PAYLOAD:
            try:
                rospy.wait_for_service('digital_output', 5)
                digital_out_service = rospy.ServiceProxy('digital_output', DigitalOutput)
                digital_output = DigitalOutputRequest()
                digital_output.digital_out = 3
                digital_output.value = False
                digital_out_service(digital_output)
            except rospy.exceptions.ROSException:
                rospy.logerr('%s, Error disabling payload.', self.name)
        else: 
            rospy.loginfo("%s: recovery action %s: INVALID ERROR CODE", self.name, error)


    def abortMission(self):
        rospy.loginfo("%s: Abort Mission", self.name)
        try:
            rospy.wait_for_service('/control_g500/disable_trajectory', 5)
            self.abort_mission_srv = rospy.ServiceProxy('/control_g500/disable_trajectory', Empty)
            self.abort_mission_srv(EmptyRequest())
        except rospy.exceptions.ROSException:
            rospy.logerr('%s, Error aborting the mission.', self.name)


    def controlledSurface(self):
        rospy.loginfo("%s, Abort mission and surface in a controlled way", self.name)
        self.abortMission()
        if not self.is_surfacing and not self.controlled_surface_enabled:
            self.controlled_surface_enabled = True
            t = Thread(target=self.controlledSurfaceThrd, args=())
            t.start()
        else:
            rospy.loginfo("%s: SURFACE is already under execution", self.name)


    def surface(self):
        rospy.loginfo("%s, Abort mission and surface", self.name)
        
        # Abort mission
        self.abortMission()
        
        # If controlled surface abort it
        self.controlled_surface_enabled = False
        
        if not self.is_surfacing:
            self.is_surfacing = True
            t = Thread(target=self.surfaceThrd, args=())
            t.start()
        else:
            rospy.loginfo("%s: SURFACE is already under execution", self.name)
    
        
    def controlledSurfaceThrd(self):
        bvr = BodyVelocityReq()
        bvr.twist.linear.x = 0.0
        bvr.twist.linear.y = 0.0
        bvr.twist.angular.x = 0.0
        bvr.twist.angular.y = 0.0
        bvr.twist.angular.z = 0.0
        bvr.disable_axis.x = False
        bvr.disable_axis.y = False
        bvr.disable_axis.z = False
        bvr.disable_axis.roll = True
        bvr.disable_axis.pitch = True
        bvr.disable_axis.yaw = False
        bvr.goal.requester = self.name
        bvr.goal.priority = GoalDescriptor.PRIORITY_EMERGENCY
        
        while self.controlled_surface_enabled:
            error_z = (self.controlled_surface_depth - self.navigation.position.depth) * 0.3
            bvr.twist.linear.z = cola2_lib.saturateValueFloat(error_z, 0.5)
            bvr.header.stamp = rospy.Time.now()
            self.pub_body_velocity_req.publish(bvr)
            rospy.sleep(0.1)
    
            
    def surfaceThrd(self):
        #TODO: Tambe es podira parar el PID i publicar directament setpoints als motors.
        bvr = BodyVelocityReq()
        bvr.twist.linear.x = 0.0
        bvr.twist.linear.y = 0.0
        bvr.twist.linear.z = -0.5
        bvr.twist.angular.x = 0.0
        bvr.twist.angular.y = 0.0
        bvr.twist.angular.z = 0.0
        bvr.disable_axis.x = False
        bvr.disable_axis.y = False
        bvr.disable_axis.z = False
        bvr.disable_axis.roll = True
        bvr.disable_axis.pitch = True
        bvr.disable_axis.yaw = False
        bvr.goal.requester = self.name
        bvr.goal.priority = GoalDescriptor.PRIORITY_EMERGENCY
        
        while self.is_surfacing:
            bvr.header.stamp = rospy.Time.now()
            self.pub_body_velocity_req.publish(bvr)
            rospy.sleep(0.1)

    
    def emergencySurface(self):
        rospy.loginfo("%s, Abort mission, surface and drop weight", self.name)
        self.abortMission()
        if not self.is_surfacing:
            self.is_surfacing = True
            t = Thread(target=self.surfaceThrd, args=())
            t.start()
        else:
            rospy.loginfo("%s: SURFACE is already under execution", self.name)
        
        rospy.loginfo("%s: Drop weight emergency system", self.name, o)
        try:
            rospy.wait_for_service('digital_output', 5)
            digital_out_service = rospy.ServiceProxy('digital_output', DigitalOutput)
            digital_output = DigitalOutputRequest()
            # TODO: Output ?? controls drop weight system
            digital_output.digital_out = 15
            digital_output.value = True
            digital_out_service(digital_output)
        except rospy.exceptions.ROSException:
            rospy.logerr('%s, Error enabling drop weight system.', self.name)
     
     
    def setZeroVelocity(self):
        r = rospy.Rate(10)
        while not rospy.is_shutdown():
            if self.navigation.position.depth > self.set_zero_velocity_depth:
                bvr = BodyVelocityReq()
                bvr.twist.linear.x = 0.0
                bvr.twist.linear.y = 0.0
                bvr.twist.linear.z = 0.0
                bvr.twist.angular.x = 0.0
                bvr.twist.angular.y = 0.0
                bvr.twist.angular.z = 0.0
                bvr.disable_axis.x = False
                bvr.disable_axis.y = False
                bvr.disable_axis.z = False
                bvr.disable_axis.roll = True
                bvr.disable_axis.pitch = True
                bvr.disable_axis.yaw = False
                bvr.goal.priority =  GoalDescriptor.PRIORITY_LOW
                bvr.goal.requester = 'set_zero_velocity'
                bvr.header.stamp = rospy.Time.now()
                self.pub_body_velocity_req.publish(bvr)
            r.sleep()


if __name__ == '__main__':
    try:
         rospy.init_node('safety_g500')
         safety_g500 = SafetyG500(rospy.get_name())
         rospy.spin()
    except rospy.ROSInterruptException: pass
