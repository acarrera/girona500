#!/usr/bin/env python

# Basic ROS imports
import roslib 
roslib.load_manifest('navigation_g500')
import rospy
import PyKDL

# import msgs
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion
from auv_msgs.msg import *
from navigation_g500.msg import *
from sensor_msgs.msg import Imu
from sensor_msgs.msg import Range
from control_g500.msg import ThrustersData

# More imports
from numpy import *
import tf

class Dynamics :

    
    def getConfig(self) :
        """ Load parameters from the rosparam server """
        self.vehicle_name = rospy.get_param('vehicle_name')
        
        self.period = rospy.get_param("dynamics/" + self.vehicle_name + "/period")
        self.mass = rospy.get_param("dynamics/" + self.vehicle_name + "/mass")
        self.gravity_center = array(rospy.get_param("dynamics/" + self.vehicle_name + "/gravity_center"))
        self.g = rospy.get_param("dynamics/" + self.vehicle_name + "/g")
        self.radius = rospy.get_param("dynamics/" + self.vehicle_name + "/radius")
        self.ctf = rospy.get_param("dynamics/" + self.vehicle_name + "/ctf")
        self.ctb = rospy.get_param("dynamics/" + self.vehicle_name + "/ctb")
        self.thrusters_tau = rospy.get_param("dynamics/" + self.vehicle_name + "/thrusters_tau")
        self.dzv = rospy.get_param("dynamics/" + self.vehicle_name + "/dzv")
        self.dv = rospy.get_param("dynamics/" + self.vehicle_name + "/dv")
        self.dh = rospy.get_param("dynamics/" + self.vehicle_name + "/dh")
        self.density = rospy.get_param("dynamics/" + self.vehicle_name + "/density")
        self.tensor = array(rospy.get_param("dynamics/" + self.vehicle_name + "/tensor"))
        self.damping = array(rospy.get_param("dynamics/" + self.vehicle_name + "/damping"))
        self.quadratic_damping = array(rospy.get_param("dynamics/" + self.vehicle_name + "/quadratic_damping"))
        
        self.p_0 = array(rospy.get_param("dynamics/" + self.vehicle_name + "/initial_pose"))
        self.v_0 = array(rospy.get_param("dynamics/" + self.vehicle_name + "/initial_velocity"))
        self.topic_name = rospy.get_param("dynamics/" + self.vehicle_name + "/topic_name")
        self.frame_id = rospy.get_param("dynamics/" + self.vehicle_name + "/frame_id")
    
#       Currents data
        self.current_mean = array( rospy.get_param("dynamics/current_mean") )
        self.current_sigma = array( rospy.get_param("dynamics/current_sigma") )
        self.current_min = array( rospy.get_param("dynamics/current_min") )
        self.current_max = array( rospy.get_param("dynamics/current_max") )
        
#       Sensors_tf
        if rospy.has_param("tritech_igc_gyro/tf") :
            imu_tf_array = array(rospy.get_param("tritech_igc_gyro/tf"))
            self.imu_tf = self.computeTf(imu_tf_array*-1.0) 
        else:
            rospy.logfatal("tritech_igc_gyro/tf param not found")
            
        if rospy.has_param("valeport_sound_velocity/tf") :
            svs_tf_array = array(rospy.get_param("valeport_sound_velocity/tf"))
            self.svs_tf = self.computeTf(svs_tf_array*-1.0) 
        else:
            rospy.logfatal("valeport_sound_velocity/tf param not found")        
        
        if rospy.has_param("teledyne_explorer_dvl/tf") :
            self.dvl_tf_array = array(rospy.get_param("teledyne_explorer_dvl/tf"))
            self.dvl_tf = self.computeTf(self.dvl_tf_array*-1.0) 
        else:
            rospy.logfatal("teledyne_explorer_dvl/tf param not found")  
            
#       Sensors period
        if rospy.has_param("tritech_igc_gyro/period") :
            self.imu_period = rospy.get_param('tritech_igc_gyro/period')
        else:
            rospy.logfatal("tritech_igc_gyro/period param not found")
            
        if rospy.has_param("valeport_sound_velocity/period") :
            self.svs_period = rospy.get_param('valeport_sound_velocity/period')
        else:
            rospy.logfatal("tritech_igc_gyro/period param not found")
            
        if rospy.has_param("teledyne_explorer_dvl/period") :
            self.dvl_period = rospy.get_param('teledyne_explorer_dvl/period')
        else:
            rospy.logfatal("teledyne_explorer_dvl/period param not found")
            
        if rospy.has_param("gps/period") :
            self.gps_period = rospy.get_param('gps/period')
        else:
            rospy.logfatal("gps/period param not found")

        if rospy.has_param("uwsim/period") :
            self.uwsim_period = rospy.get_param('uwsim/period')
        else:
            rospy.logfatal("uwsim/period param not found")


#       Sensors covariance
        if rospy.has_param("tritech_igc_gyro/orientation_covariance") :
            self.imu_orientation_covariance = array(rospy.get_param('tritech_igc_gyro/orientation_covariance'))
        else:
            rospy.logfatal("tritech_igc_gyro/orientation_covariance param not found")
            
        if rospy.has_param("valeport_sound_velocity/pressure_covariance") :
            self.svs_pressure_covariance = rospy.get_param('valeport_sound_velocity/pressure_covariance')
        else:
            rospy.logfatal("valeport_sound_velocity/pressure_covariance param not found")
            
        if rospy.has_param("teledyne_explorer_dvl/velocity_covariance") :
            self.dvl_velocity_covariance = array(rospy.get_param('teledyne_explorer_dvl/velocity_covariance'))
        else:
            rospy.logfatal("teledyne_explorer_dvl/velocity_covariance param not found")
        
        if rospy.has_param("gps/position_covariance") :
            self.gps_position_covariance = rospy.get_param('gps/position_covariance')
        else:
            rospy.logfatal("gps/position_covariance param not found")
        
        
        
    def s(self, x) :
        """ Given a 3D vector computes a 3x3 matrix for .... ? """
#        rospy.loginfo("s(): \n %s", x)
        ret = array([0.0, -x[2], x[1], x[2], 0.0, -x[0], -x[1], x[0], 0.0 ])
        return ret.reshape(3,3)


    def generalizedForce(self, du):
        # Build the signed (lineal/quadratic) thruster coeficient array
        # Signed square of each thruster setpoint
        du = du * abs(du)
    
        ct = zeros(len(du))
        i1 = nonzero(du >= 0.0)
        i2 = nonzero(du <= 0.0)
        ct[i1] = self.ctf
        ct[i2] = self.ctb
        
        #Odin Thrusters
#        b = [ct[0],     0,          ct[2],      0,          0,          0,          0,          0,
#             0,         -ct[1],     0,          -ct[3],     0,          0,          0,          0,
#             0,         0,          0,          0,          -ct[4],     -ct[5],     -ct[6],     -ct[7],
#             cq,        0,          cq,         0,          ct[4]*dv,   0,          -ct[6]*dv,  0,
#             0,         -cq,        0,          -cq,        0,          -ct[5]*dv,  0,          ct[7]*dv,
#             ct[0]*dh,  ct[1]*dh,   -ct[2]*dh,  -ct[3]*dh,  -cq,        -cq,        -cq,        -cq]
        
        #Garbi/Girona500 Thrusters
        b = [-ct[0],            -ct[1],             .0,             .0,             .0,
             .0,                .0,                 .0,             .0,             ct[4],
             .0,                .0,                 -ct[2],         -ct[3],         .0,
             .0,                .0,                 .0,             .0,             .0,
             .0,                .0,                 -ct[2]*self.dv, ct[3]*self.dv,  .0,
             -ct[0]*self.dh,     ct[1]*self.dh,     .0,             .0,             .0]
        b = array(b).reshape(6,5)
      
        # t = generalized force
        t = dot(b, du)
        t = squeeze(asarray(t)) #Transforms a matrix into an array
        return t


    def coriolisMatrix(self):
        s1 = self.s(dot(self.M[0:3,0:3], self.v[0:3]) + dot(self.M[0:3,3:6], self.v[3:6]))
        s2 = self.s(dot(self.M[3:6,0:3], self.v[0:3]) + dot(self.M[3:6,3:6], self.v[3:6])) 
        c = zeros((6, 6))
        c[0:3,3:6] = -s1
        c[3:6,0:3] = -s1
        c[3:6,3:6] = -s2
        return c
    
    def dumpingMatrix(self):
        # lineal hydrodynamic damping coeficients  
        Xu = self.damping[0]
        Yv = self.damping[1]
        Zw = self.damping[2]
        Kp = self.damping[3]
        Mq = self.damping[4]
        Nr = self.damping[5]
        
        # quadratic hydrodynamic damping coeficients
        Xuu = self.quadratic_damping[0]    #[Kg/m]
        Yvv = self.quadratic_damping[1]    #[Kg/m]
        Zww = self.quadratic_damping[2]    #[Kg/m]
        Kpp = self.quadratic_damping[3]    #[Kg*m*m]
        Mqq = self.quadratic_damping[4]    #[Kg*m*m]
        Nrr = self.quadratic_damping[5]    #[Kg*m*m]
    
        d = diag([Xu + Xuu*abs(self.v[0]), 
                  Yv + Yvv*abs(self.v[1]),
                  Zw + Zww*abs(self.v[2]),
                  Kp + Kpp*abs(self.v[3]),
                  Mq + Mqq*abs(self.v[4]),
                  Nr + Nrr*abs(self.v[5])])
        return d

    def gravity(self):
        #Weight and Flotability
        W = self.mass * self.g # [Kg]
        
        #If the vehicle moves out of the water the flotability decreases
        if self.p[2] < 0.0: 
            r = self.radius + self.p[2]
            if r < 0.0:
                r = 0.0
        else :
            r = self.radius
            
        F = ((4 * math.pi * pow(r,3))/3)*self.density*self.g 
  
        # gravity center position in the robot fixed frame (x',y',z') [m]
        zg = self.gravity_center[2]
        
        g = array([(W - F) * sin(self.p[4]),
                   -(W - F) * cos(self.p[4]) * sin(self.p[3]),
                   -(W - F) * cos(self.p[4]) * cos(self.p[3]),
                   zg*W*cos(self.p[4])*sin(self.p[3]),
                   zg*W*sin(self.p[4]),
                   0.0])
        
        return g
        
        
    def inverseDynamic(self) :
        """ Given the setpoint for each thruster, the previous velocity and the 
            previous position computes the v_dot """
            
        du = self.thrustersDynamics(self.u)
        t = self.generalizedForce(du)
        c = self.coriolisMatrix()
        d = self.dumpingMatrix()
        g = self.gravity()
        c_v = dot((c-d), self.v)
        v_dot = dot(self.IM, (t-c_v-g)) #t-c_v-g
        v_dot = squeeze(asarray(v_dot)) #Transforms a matrix into an array
        return v_dot
        
#
    
    def integral(self, x_dot, x, t) :
        """ Computes the integral o x dt """
        return (x_dot * t) + x
    
    
    def kinematics(self) :
        """ Given the current velocity and the previous position computes the p_dot """
        roll = self.p[3]
        pitch = self.p[4]
        yaw = self.p[5]
        
        rec = [cos(yaw)*cos(pitch), -sin(yaw)*cos(roll)+cos(yaw)*sin(pitch)*sin(roll), sin(yaw)*sin(roll)+cos(yaw)*cos(roll)*sin(pitch),
               sin(yaw)*cos(pitch), cos(yaw)*cos(roll)+sin(roll)*sin(pitch)*sin(yaw), -cos(yaw)*sin(roll)+sin(pitch)*sin(yaw)*cos(roll),
               -sin(pitch), cos(pitch)*sin(roll), cos(pitch)*cos(roll)]
        rec = array(rec).reshape(3,3)
        
        to = [1.0, sin(roll)*tan(pitch), cos(roll)*tan(pitch),
              0.0, cos(roll), -sin(roll),
              0.0, sin(roll)/cos(pitch), cos(roll)/cos(pitch)]
        to = array(to).reshape(3,3)
        
        p_dot = zeros(6)
        p_dot[0:3] = dot(rec, self.v[0:3])
        p_dot[3:6] = dot(to, self.v[3:6])
        return p_dot

    
    def updateThrusters(self, thrusters) :
        t = array(thrusters.setpoints)
        i = nonzero(t > 1.0)
        t[i] = 1.0
        i = nonzero(t < -1.0)
        t[i] = -1.0
        self.u = t*1500.0
        
        
    def updateAltitude(self, range) :
        #rospy.loginfo("Range:\n%s", range)
        self.altitude = range.range
        
        
    def thrustersDynamics(self, u):
        y = zeros(size(u))
        for i in range(size(u)):
            y[i] = (self.period * u[i] + self.thrusters_tau * self.y_1[i]) / (self.period + self.thrusters_tau)
            
        self.y_1 = y
        return y
    
    
    def pubOdometry(self, event):
        odom = Odometry()
        odom.header.stamp = rospy.Time.now()
        odom.header.frame_id = str(self.frame_id)
        odom.child_frame_id = "real_girona500"
        
        odom.pose.pose.position.x = self.p[0]
        odom.pose.pose.position.y = self.p[1]
        odom.pose.pose.position.z = self.p[2]
        
        orientation = tf.transformations.quaternion_from_euler(self.p[3], self.p[4], self.p[5], 'sxyz')
        odom.pose.pose.orientation.x = orientation[0]
        odom.pose.pose.orientation.y = orientation[1]
        odom.pose.pose.orientation.z = orientation[2]
        odom.pose.pose.orientation.w = orientation[3]
            
        #Haig de comentar les velocitats sino l'UWSim no va 
        odom.twist.twist.linear.x = 0.0 #v_[0]
        odom.twist.twist.linear.y = 0.0 #v_[1]
        odom.twist.twist.linear.z = 0.0 #v_[2]
        odom.twist.twist.angular.x = 0.0 #v_[3]
        odom.twist.twist.angular.y = 0.0 #v_[4]
        odom.twist.twist.angular.z = 0.0 #v_[5]
    
        self.pub_odom.publish(odom)
     
        # Broadcast transform
        br = tf.TransformBroadcaster()
        br.sendTransform((self.p[0], self.p[1], self.p[2]), orientation, 
        odom.header.stamp, odom.header.frame_id, "world")
    
    
    def pubNavSts(self):
        nav_sts = NavSts()
        nav_sts.header.stamp = rospy.Time.now()
        nav_sts.header.frame_id = str(self.frame_id)
        nav_sts.position.north = self.p[0]
        nav_sts.position.east = self.p[1]
        nav_sts.position.depth = self.p[2]
        nav_sts.altitude = self.altitude
        nav_sts.body_velocity.x = self.v[0]
        nav_sts.body_velocity.y = self.v[1]
        nav_sts.body_velocity.z = self.v[2]
        nav_sts.orientation.roll = self.p[3]
        nav_sts.orientation.pitch = self.p[4]
        nav_sts.orientation.yaw = self.p[5]
        nav_sts.orientation_rate.roll = self.v[3]
        nav_sts.orientation_rate.pitch = self.v[4]
        nav_sts.orientation_rate.yaw = self.v[5]
        nav_sts.status = nav_sts.STATUS_ALL_OK
#        self.pub_nav_sts.publish(nav_sts)
    
    
    #TODO: Aquesta funcio esta duplicada al merged_navigation        
    def computeTf(self, tf):
        r = PyKDL.Rotation.RPY(math.radians(tf[3]), math.radians(tf[4]), math.radians(tf[5]))
        v = PyKDL.Vector(tf[0], tf[1], tf[2])
        frame = PyKDL.Frame(r, v)
        return frame
    
                    
    def pubImu(self, event):
        imu = Imu()
        imu.header.stamp = rospy.Time.now()
        imu.header.frame_id = str(self.frame_id)
        roll = self.p[3] + random.normal(0.0, self.imu_orientation_covariance[0])
        pitch = self.p[4] + random.normal(0.0, self.imu_orientation_covariance[1])
        yaw = self.p[5] + random.normal(0.0, self.imu_orientation_covariance[2])
        
        vehicle_rpy = PyKDL.Rotation.RPY(roll, pitch, yaw)
        #TODO: Is this right? Maybe vehicle_rpy * self.imu_tf.M instead???
        imu_orientation = self.imu_tf.M * vehicle_rpy
        angle = imu_orientation.GetRPY()
        angle = tf.transformations.quaternion_from_euler(angle[0], angle[1], angle[2])
        imu.orientation.x = angle[0]
        imu.orientation.y = angle[1]
        imu.orientation.z = angle[2]
        imu.orientation.w = angle[3]
        
        imu.orientation_covariance[0] = self.imu_orientation_covariance[0]
        imu.orientation_covariance[4] = self.imu_orientation_covariance[1]
        imu.orientation_covariance[8] = self.imu_orientation_covariance[2]
        
        self.pub_imu.publish(imu)
        
        
    def pubSvs(self, event):
        svs = ValeportSoundVelocity()
        svs.header.stamp = rospy.Time.now()
        svs.header.frame_id = str(self.frame_id)
        vehicle_rpy = PyKDL.Rotation.RPY(self.p[3], self.p[4], self.p[5])
        svs_data = vehicle_rpy * self.svs_tf.p
        svs.pressure = svs_data[2] + self.p[2] + random.normal(0.0, self.svs_pressure_covariance)
        self.pub_svs.publish(svs)
        
        
    def pubDvl(self, event):
        dvl = TeledyneExplorerDvl()
        dvl.header.stamp = rospy.Time.now()
        dvl.header.frame_id = str(self.frame_id)
        
        #add noise
        v_dvl = zeros(3)
        v_dvl[0] = self.v[0] + random.normal(0.0, self.dvl_velocity_covariance[0])
        v_dvl[1] = self.v[1] + random.normal(0.0, self.dvl_velocity_covariance[1])
        v_dvl[2] = self.v[2] + random.normal(0.0, self.dvl_velocity_covariance[2])
        
        #velocity is computed add the gravity center but we want the velocity at the sensor
        #Vdvl = Vg500 + (v.ang.z x dist(dvl->g500))
        angular_velocity = array([self.v[3], self.v[4], self.v[5]])
        distance = self.dvl_tf_array[0:3]
        
        #TODO: CHECK IF--> v_dvl = v_dvl - cross(angular_velocity, distance) instead
        v_dvl = v_dvl + cross(angular_velocity, distance)
        
        #The DVL is not dextrogir
        v_dvl[1] = -v_dvl[1]

        #Rotate the DVL        
        dvl_data = PyKDL.Vector(v_dvl[0], v_dvl[1], v_dvl[2])
        dvl_data = self.dvl_tf.M * dvl_data
        dvl.bi_x_axis = dvl_data[0]
        dvl.bi_y_axis = dvl_data[1]
        dvl.bi_z_axis = dvl_data[2]
        dvl.bi_status = "A"
        dvl.bd_range = self.altitude
        self.pub_dvl.publish(dvl)
        
        
    def pubGps(self, event):
        #Publish GPS data only near to surface
        if self.p[2] < 0.5 :
            gps = FastraxIt500Gps()
            gps.header.stamp = rospy.Time.now()
            gps.header.frame_id = str(self.frame_id)
            north = self.p[0] + random.normal(0.0, self.gps_position_covariance[0])
            east = self.p[1] + random.normal(0.0, self.gps_position_covariance[1])
            gps.north = north
            gps.east = east
            gps.data_quality = 1
            self.pub_gps.publish(gps)
        
        
    def __init__(self):
        """ Simulates the dynamics of an AUV """
    
    #   Load dynamic parameters
        self.getConfig()
        self.altitude = -1.0 
        self.y_1 = zeros(5)
        
    #   Create publisher
        self.pub_odom = rospy.Publisher(str(self.topic_name), Odometry)
##       self.pub_nav_sts = rospy.Publisher('/navigation_g500/nav_sts', NavSts)
        self.pub_imu = rospy.Publisher('/navigation_g500/imu', Imu)
        self.pub_svs = rospy.Publisher('/navigation_g500/valeport_sound_velocity', ValeportSoundVelocity)
        self.pub_dvl = rospy.Publisher('/navigation_g500/teledyne_explorer_dvl', TeledyneExplorerDvl)
        self.pub_gps = rospy.Publisher('/navigation_g500/fastrax_it_500_gps', FastraxIt500Gps)
        rospy.init_node('dynamics')
                
        
    #   Init pose and velocity and period
        self.v = self.v_0
        self.p = self.p_0
        
        # Inertia Tensor. Principal moments of inertia, and products of inertia [kg*m*m]
        Ixx = self.tensor[0]
        Ixy = self.tensor[1] 
        Ixz = self.tensor[2]
        Iyx = self.tensor[3]
        Iyy = self.tensor[4]
        Iyz = self.tensor[5]
        Izx = self.tensor[6]
        Izy = self.tensor[7]
        Izz = self.tensor[8] 
        m = self.mass
        xg = self.gravity_center[0]
        yg = self.gravity_center[1]
        zg = self.gravity_center[2]
        
        Mrb=[m,     0,      0,      0,      m*zg,       -m*yg,
             0,     m,      0,      -m*zg,  0,          m*xg,
             0,     0,      m,      m*yg,   -m*xg,      0,
             0,     -m*zg,  m*yg,   Ixx,    Ixy,        Ixz,
             m*zg,  0,      -m*xg,  Iyx,    Iyy,        Iyz,
             -m*yg, m*xg,   0,      Izx,    Izy,        Izz]
        Mrb = array(Mrb).reshape(6, 6)
             
        # Inertia matrix of the rigid body
        # Added Mass derivative
        Ma=[m/2,    0,      0,      0,      0,      0,
            0,      m/2,    0,      0,      0,      0,
            0,      0,      m/2,    0,      0,      0,
            0,      0,      0,      0,      0,      0,
            0,      0,      0,      0,      0,      0,
            0,      0,      0,      0,      0,      0]
        Ma = array(Ma).reshape(6, 6) 
        
        self.M = Mrb + Ma    # mass matrix: Mrb + Ma
        self.IM = matrix(self.M).I
#        rospy.loginfo("Inverse Mass Matrix: \n%s", str(self.IM))
              
    #   Init currents
        random.seed()
        self.e_vc = self.current_mean 
        self.u = array([0.0, 0.0, 0.0, 0.0, 0.0]) # Initial thrusters setpoint

    #   Init simulated sensor
        rospy.Timer(rospy.Duration(self.imu_period), self.pubImu)
        rospy.Timer(rospy.Duration(self.svs_period), self.pubSvs)
        rospy.Timer(rospy.Duration(self.dvl_period), self.pubDvl)
        rospy.Timer(rospy.Duration(self.gps_period), self.pubGps)
        rospy.Timer(rospy.Duration(self.uwsim_period), self.pubOdometry)
        
    #   Create Subscriber
        rospy.Subscriber("/control_g500/thrusters_data", ThrustersData, self.updateThrusters)
        rospy.Subscriber("/uwsim/g500/range", Range, self.updateAltitude)
        

    def iterate(self):
        t1 = rospy.Time.now()

        # Main loop operations
        self.v_dot = self.inverseDynamic()
        self.v = self.integral(self.v_dot, self.v, self.period)
        self.p_dot = self.kinematics()
        self.p = self.integral(self.p_dot, self.p, self.period)

        t2 = rospy.Time.now()
        p = self.period - (t2-t1).to_sec()
        if p < 0.0 : p = 0.0
        rospy.sleep(p)
        

if __name__ == '__main__':
    try:
        dynamics = Dynamics() 
        while not rospy.is_shutdown():
            dynamics.iterate()

    except rospy.ROSInterruptException: pass
    
