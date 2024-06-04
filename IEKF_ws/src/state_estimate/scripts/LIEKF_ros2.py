#!/usr/bin/env python3

import torch
import rospy
from sensor_msgs.msg import Imu
from std_msgs.msg import String
from geometry_msgs.msg import TransformStamped
from geometry_msgs.msg import Vector3Stamped
from geometry_msgs.msg import QuaternionStamped
from geometry_msgs.msg import Quaternion
from scipy.spatial.transform import Rotation as Rotlib
import pymap3d as pm
from state_estimate.msg import state_IMU
from time_synchronization.msg import sync_gather
import numpy as np
import pandas as pd

#function definition
def Quat2Rot(Quaternion):
    q0 = Quaternion[3] #w
    q1 = Quaternion[0] #x
    q2 = Quaternion[1] #y
    q3 = Quaternion[2] #z
    R = torch.tensor([[1-2*(q2*q2+q3*q3),  2*(q1*q2-q0*q3),  2*(q0*q2+q1*q3)],
                      [2*(q1*q2+q0*q3),    1-2*(q1*q1+q3*q3),2*(q2*q3-q0*q1)],
                      [2*(q1*q3-q0*q2),    2*(q2*q3+q0*q1),   1-2*(q1*q1+q2*q2)]])
    return R

#IEKF class definition
class LIEKF:
    # parameters
    
    g = torch.tensor([0,0,-9.80665])
    P_dim = 21
    Q_dim = 21
    N_dim = 3
    Id2 = torch.eye(2)
    Id3 = torch.eye(3)
    IdP = torch.eye(21)
    Q = torch.zeros([21,21])
    N_n_c = torch.zeros([3,3])
    P = torch.zeros([21,21])
    z_std = torch.zeros([3])
        
    cov_omega = 1e-3                # gyro covariance
    cov_acc = 1e-2                  # accelerometer covariance
    cov_b_omega = 6e-9              # gyro bias covariance
    cov_b_acc = 2e-4                # accelerometer bias covariance
    cov_Rot_c_i = 1e-9              # car to IMU orientation covariance
    cov_t_c_i = 1e-9                # car to IMU translation covariance

    # GPS-measurment covariance
    var_lat = 0.2                 # Zero lateral velocity variance
    beta_lat = 3                    # scale factor var_lat for covariance
    var_up = 300                  # Zero upward velocity variance
    beta_up = 3                     # scale factor var_up for covariance

    # State covariance
    cov_Rot0 = 1e-3                 # initial pitch and roll covariance
    cov_b_omega0 = 1e-1             # initial gyro bias covariance
    cov_b_acc0 = 1e-1               # initial accelerometer bias covariance
    cov_v0 = 1e-2                   # initial velocity covariance
    cov_p0 = 1e-2                   # initial position covariance
    cov_Rot_c_i0 = 1e-3             # initial car to IMU pitch and roll covariance
    cov_t_c_i0 = 5e-3               # initial car to IMU translation covariance
    

    # numerical parameters
    n_normalize_rot = 100           # timestamp before normalizing orientation
    n_normalize_rot_c_i = 1000      # timestamp before normalizing car to IMU orientation
    
    
    # matrix initialization
    N = 655350
    X = torch.zeros((N,5,5))
    Rot = torch.zeros((N, 3, 3))                                # Rotation matrix (orientation) in car-frame
    v = torch.zeros((N, 3))                                     # Velocity vector
    p = torch.zeros((N, 3))                                     # Position vector
    b_omega = torch.zeros((N, 3))                               # Angular speed biase
    b_acc = torch.zeros((N, 3))                                 # Acceleration biase
    R_c = torch.zeros((N, 3, 3))                            # Rotation matrix from car-frame to IMU-frame
    p_c = torch.zeros((N, 3))                                 # Translation vector from car-frame to IMU-frame

    R_b = torch.zeros((N,3,3))                                # Rotation matrix from encoder-frame to IMU-frame
    p_b = torch.zeros((N,3))                                  # Translation vector from encoder-frame to IMU-frame
    
    z = torch.zeros([N,3])

    t = torch.zeros([N])
    P_store = torch.zeros([N, 21, 21])
    u = torch.zeros([6,N])
    p_GPS = torch.zeros([3])
    p_GPS_LLA = torch.zeros([3],dtype=torch.float64)
    p_GPS_LLA0 = torch.zeros([3],dtype=torch.float64)
    rot_euler = torch.zeros([N,3])
    p_world = torch.zeros([N,3])
    
    omega = torch.zeros([3])
    # position and Orientation initial parameter
    vicon_quat0 = torch.zeros([4])
    vicon_pos0 = torch.zeros([3])
    
    # imu calibration parameter initialization
    omega_init_mean = torch.zeros([3])
    acc_init_mean = torch.zeros([3])
    p_GPS_LLA_init_mean = torch.zeros([100,3],dtype=torch.float64)
    imu_init_index = 0
    t_last = 0
    imu_cal_flag = 0
    run_finish_flag = 1
    # time index after init_protocol
    time_index = 0

    # tmp variables
    u_tmp = torch.zeros([6])
    p_GPS_tmp = torch.zeros([3])
    p_GPS_LLA_tmp = torch.zeros([3],dtype=torch.float64)
    # ros publish
    pub = rospy.Publisher('state_estimate',state_IMU,queue_size=10)

    # vicon comparison
    quat = torch.zeros([4])
    quat[3] = 1
    
    # configure time interval
    dt = 0.02
    dt_real = 0

    # way points coordinate
    p_way_point_LLA = None # LLA absolute
    p_way_point = None # ENU relative

    #functions
    def __init__(self):
        #waypoints
        self.p_way_point_LLA = pd.read_csv('/home/bikebot/IEKF_ws/src/state_estimate/scripts/waypoint.csv')
        self.p_way_point_LLA = torch.tensor(self.p_way_point_LLA.values,dtype=torch.float64)
        self.p_way_point_LLA = self.p_way_point_LLA[:,1:]
        
        self.p_way_point = torch.zeros_like(self.p_way_point_LLA, dtype=torch.float32)
        print(self.p_way_point_LLA.shape[0])

        #EKF parameters

        self.Q = torch.diag(torch.Tensor([self.cov_omega,       self.cov_omega,     self. cov_omega,         # Set the state noise matix
                                    self.cov_acc,         self.cov_acc,       self.cov_acc,
                                    0           ,         0           ,       0           ,
                                    self.cov_b_omega,     self.cov_b_omega,   self.cov_b_omega,
                                    self.cov_b_acc,       self.cov_b_acc,     self.cov_b_acc,
                                    self.cov_Rot_c_i,     self.cov_Rot_c_i,   self.cov_Rot_c_i,
                                    self.cov_t_c_i,       self.cov_t_c_i,     self.cov_t_c_i]))
        
        self.P[:3, :3] = self.cov_Rot0 * self.Id3                        # Set initial orientation covariance, with no error on initial yaw value
        self.P[3:6, 3:6] = self.cov_v0 * self.Id3                        # Set initial velocity (x and y) covariance
        
        self.P[6:9, 6:9] = self.cov_p0 * self.Id3                        # Set initial z velocity and position covariance to 0
        self.P[9:12, 9:12] = self.cov_b_omega0 * self.Id3                # Set initial angular speed biase covariance
        self.P[12:15, 12:15] = self.cov_b_acc0 * self.Id3                # Set initial acceleration biase covariance
        self.P[15:18, 15:18] = self.cov_Rot_c_i0 * self.Id3              # Set initial rotation car to IMU frame covariance
        self.P[18:21, 18:21] = self.cov_t_c_i0 * self.Id3                # Set initial translation car to IMU frame covariance
        
        self.N_n_c = torch.diag(torch.tensor([0.1,0.1,0.1]))
             
    def run(self):
        """
        Run IEKF algorithm on input sequence
        :param t: time vector
        :param u: input measurement, u = [wx, wy, wz,ax, ay, az]
        :param z_covs: pseudo-measure covariance, [cov_v_lat, cov_v_up]
        :param v_0: initial velocity
        :param ang0: initial orientation
        :return:
        """
        
        # Initialise the states variables with initial condictions
        if (self.time_index >= 1):
            #self.t[self.time_index] = self.t_last
            
            _Last = rospy.Time.now().to_sec()

            _Now = rospy.Time.now().to_sec()
            
            while (_Now-_Last < self.dt):
                _Now = rospy.Time.now().to_sec()
            
            while not rospy.is_shutdown():
                _Last = rospy.Time.now().to_sec()
                print('estimating...')
                
                dt = self.dt_real
                #print(self.time_index)
                self.u[:,self.time_index] = self.u_tmp.clone()
                #print(self.u[:,self.time_index])
                self.p_GPS = self.p_GPS_tmp.clone()
                
                X_propag,b_omega_propag,b_acc_propag,R_c_propag,p_c_propag,   P_propag = \
                    self.propagate(self.P_store[self.time_index-1],self.u[:3,self.time_index],self.u[3:,self.time_index],self.X[self.time_index-1],dt,self.b_omega[self.time_index-1],self.b_acc[self.time_index-1],self.R_c[self.time_index-1],self.p_c[self.time_index-1])
                self.X[self.time_index], self.b_omega[self.time_index], self.b_acc[self.time_index], self.R_c[self.time_index], self.p_c[self.time_index], self.P_store[self.time_index,:,:],self.z[self.time_index,:],self.p_world[self.time_index,:]= \
                    self.state_covariance_update(X_propag,b_omega_propag,b_acc_propag,R_c_propag,p_c_propag,P_propag,self.N_n_c,self.p_GPS)
                
                self.p[self.time_index,:] = self.X[self.time_index][:3,4]
                self.v[self.time_index,:] = self.X[self.time_index][:3,3]
                self.Rot[self.time_index,:,:] = self.X[self.time_index][:3,:3]
                rot = Rotlib.from_matrix(self.Rot[self.time_index,:,:])
                a = rot.as_euler('zxy',degrees=True)
                rot_tmp = np.array(a)
                self.rot_euler[self.time_index,:] = torch.tensor(rot_tmp)

                # correct numerical error every second
                if self.time_index % self.n_normalize_rot == 0:
                    self.Rot[self.time_index] = self.normalize_rot(self.Rot[self.time_index])
                # correct numerical error every 10 seconds
                if self.time_index % self.n_normalize_rot_c_i == 0:
                    self.R_c[self.time_index] = self.normalize_rot(self.R_c[self.time_index])

                quat_vicon = Rotlib.from_quat(self.quat)
                euler_vicon = quat_vicon.as_euler('zxy',degrees=True)

                msg = state_IMU()
                msg.header.stamp = rospy.Time.now()
                msg.pos = [float(self.p[self.time_index,0]),float(self.p[self.time_index,1]),float(self.p[self.time_index,2])]
                msg.vel = [float(self.v[self.time_index,0]),float(self.v[self.time_index,1]),float(self.v[self.time_index,2])]
                msg.psi = [float(self.rot_euler[self.time_index,0]),float(self.rot_euler[self.time_index,1]),float(self.rot_euler[self.time_index,2])]   
                msg.psi_vicon = [float(euler_vicon[0]),float(euler_vicon[1]),float(euler_vicon[2])]
                msg.transform.translation.x = self.p_GPS[0]
                msg.transform.translation.y = self.p_GPS[1]
                msg.transform.translation.z = self.p_GPS[2]  
                msg.error_vec = [float(self.z[self.time_index,0]),float(self.z[self.time_index,1]),float(self.z[self.time_index,2])]
                msg.error_2D = float(torch.sqrt(self.z[self.time_index,0]**2+self.z[self.time_index,1]**2))
                msg.pos_vicon = self.p_world.tolist()+self.p_way_point.flatten().tolist()

                msg.omega = [float(self.omega[0]),float(self.omega[1]),float(self.omega[2])]
                self.pub.publish(msg)
                _Now = rospy.Time.now().to_sec()

                #every processing at this time step is done, index to next time step    
                self.time_index += 1

                while (_Now-_Last < self.dt):
                    _Now = rospy.Time.now().to_sec()
                self.dt_real = _Now-_Last
            rospy.spin()
                

            
        
    #functions
    def vicon_quaternion_callback(self,data):
        self.quat[0] = data.quaternion.x
        self.quat[1] = data.quaternion.y
        self.quat[2] = data.quaternion.z
        self.quat[3] = data.quaternion.w
    


    def init_protocol_callback(self,data):
        """"
        when receiving initial command from ros, start intialization
        """
        rospy.loginfo('start initial protocol...')
        #self.Rot[0,:,:] = Quat2Rot(self.vicon_quat0)
        
        #self.v[0,:] = v_mes0
        self.v[0,:] = torch.tensor([0,0,0])
        self.R_c[0] = self.Id3          # Set initial rotation between car and IMU frame to 0, i.e. identity matrix  
        #self.p_c[0] = torch.tensor([-0.110-0.006253,(0.102+0.01175),(0.081-0.007645)])
        self.p_c[0] = torch.tensor([-117.380811e-3,-517.9640275e-3,-27.2120583e-3])
        
        
        rospy.Subscriber('/sync/gather',sync_gather,self.IMU_init_callback)    #need to confirm
        while self.imu_init_index < 100:
            pass
        rospy.loginfo('Done IMU bias calibration...')
        
        self.omega_init_mean /= 100
        self.acc_init_mean /= 100
        quat0 = Rotlib.from_quat(self.quat)
        tmp_Rot = quat0.as_matrix()
        self.Rot[0,:,:] = torch.tensor(tmp_Rot)
        #print('Rot0',self.Rot[0,:,:])
        if self.time_index == 0:
            #initial position acquire
            self.p_GPS_LLA0[0] = torch.mean(self.p_GPS_LLA_init_mean[:,0],dtype=torch.float64)
            self.p_GPS_LLA0[1] = torch.mean(self.p_GPS_LLA_init_mean[:,1],dtype=torch.float64)
            self.p_GPS_LLA0[2] = torch.mean(self.p_GPS_LLA_init_mean[:,2],dtype=torch.float64)
            
            #waypoints position relative to initial, ENU
            for index in range(self.way_point_LLA.shape[0]):
                self.p_way_point[index,:] = \
                    pm.geodetic2enu(self.p_way_point_LLA[index,0],self.p_way_point_LLA[index,1],self.p_way_point_LLA[index,2],\
                                                          self.p_GPS_LLA0[0],self.p_GPS_LLA0[1],self.p_GPS_LLA0[2])
            


        
        self.b_omega[0] = self.omega_init_mean
        #print('b_omega0',self.b_omega[0])
        self.b_acc[0] = self.acc_init_mean + self.g 
        #print('b_acc0',self.b_acc[0])
        self.p[0,:] = self.Rot[0,:,:].mv(self.p_c[0])
        self.X[0,:3,:3] = self.Rot[0]
        self.X[0,:3,3] = self.v[0,:].clone()
        self.X[0,:3,4] = self.p[0,:].clone()
        self.X[0,3,3] = 1
        self.X[0,4,4] = 1   
        self.P_store[0,:,:] = self.P.clone()
        self.t[0] = self.t_last
        self.time_index = 1
        self.imu_cal_flag = 1

        
    def vicon_run_callback(self,data):
        if self.imu_cal_flag == 0 :
            self.vicon_quat0 = torch.tensor([data.transform.rotation.x,data.transform.rotation.y,data.transform.rotation.z,data.transform.rotation.w])

        self.t[self.time_index] = data.header.stamp.secs + data.header.stamp.nsecs * 1e-9
        self.u_tmp[0] = data.angular_velocity.x
        self.u_tmp[1] = data.angular_velocity.y
        self.u_tmp[2] = data.angular_velocity.z
        self.u_tmp[3] = data.linear_acceleration.x
        self.u_tmp[4] = data.linear_acceleration.y
        self.u_tmp[5] = data.linear_acceleration.z


      #calculate current position relative to initial calibrating point
        self.p_GPS_LLA_tmp[0] = data.transform.translation.x
        self.p_GPS_LLA_tmp[1] = data.transform.translation.y
        self.p_GPS_LLA_tmp[2] = data.transform.translation.z
        #print('LLA0',self.p_GPS_LLA0)
        tmp = (pm.geodetic2enu(self.p_GPS_LLA_tmp[0],self.p_GPS_LLA_tmp[1],self.p_GPS_LLA_tmp[2],self.p_GPS_LLA0[0],self.p_GPS_LLA0[1],self.p_GPS_LLA0[2]))

        self.p_GPS_tmp[0] = tmp[0]
        self.p_GPS_tmp[1] = tmp[1]
        self.p_GPS_tmp[2] = tmp[2]


        
        
        
    def IMU_init_callback(self,data):
        
        #print(self.acc_init_mean)


        if self.imu_init_index<100:
            self.p_GPS_LLA_init_mean[self.imu_init_index,0] = data.transform.translation.x
            self.p_GPS_LLA_init_mean[self.imu_init_index,1] = data.transform.translation.y
            self.p_GPS_LLA_init_mean[self.imu_init_index,2] = data.transform.translation.z

            self.omega_init_mean[0] += data.angular_velocity.x
            self.omega_init_mean[1] += data.angular_velocity.y
            self.omega_init_mean[2] += data.angular_velocity.z  
        
            self.acc_init_mean[0] += data.linear_acceleration.x
            self.acc_init_mean[1] += data.linear_acceleration.y
            self.acc_init_mean[2] += data.linear_acceleration.z
        #print(self.p_GPS_LLA)
        self.t_last = data.header.stamp.secs + data.header.stamp.nsecs * 1e-9
        
        self.imu_init_index += 1 
    def propagate(self,P_prev,omega_prev,acc_prev,X_prev,dt,b_omega_prev,b_acc_prev,R_c_prev,p_c_prev):
        #SE2(3) state matrix: [R_3,3  v_3,1 p_3,1
        #                      0_1,3  1     0
        #                      0_1,3  0     1     ]
        
        
        Propag_mat = torch.zeros([5,5])
        Rot_prev = X_prev[:3,:3].clone()
        v_prev = X_prev[:3,3].clone()
        p_prev = X_prev[:3,4].clone()

        self.omega = omega_prev-b_omega_prev
        Rot_propag = (self.so3_alg2gro((omega_prev-b_omega_prev) * dt))
        v_propag  = (Rot_prev.mv(acc_prev-b_acc_prev) + self.g)
        p_propag = v_prev + 1/2*(acc_prev-b_acc_prev)*dt
        
        #define propagation matrix
        Propag_mat[:3,:3] = Rot_propag.clone()
        Propag_mat[:3,3] = v_propag.clone() * dt
        Propag_mat[:3,4] = p_propag.clone() * dt
        
        
        #propagate state
        X_propag = torch.eye(5)
        X_propag[:3,:3] = Rot_prev.mm(Rot_propag)
        X_propag[:3,3] = v_prev+Propag_mat[:3,3]
        X_propag[:3,4] = p_prev+Propag_mat[:3,4]
        
        #other state variable propagate (just clone)
        b_omega_propag = b_omega_prev
        b_acc_propag = b_acc_prev
        R_c_propag = R_c_prev.clone()
        p_c_propag = p_c_prev
        
        #covariance propagate
        F =  torch.zeros([self.P_dim, self.P_dim])
        #AD_x = torch.zeros([self.P_dim,self.Q_dim])
        Q = self.Q.clone()
        
        #v_skew_rot = self.skew_sym_mat(v_prev).mm(Rot_prev)
        #p_skew_rot = self.skew_sym_mat(p_prev).mm(Rot_prev)
        
        #fill F matrix
        F[:3,:3] = -self.skew_sym_mat((omega_prev-b_omega_prev))
        F[:3,9:12] = -self.Id3
        F[3:6,:3] = -self.skew_sym_mat((acc_prev-b_acc_prev))
        F[3:6,3:6] = -self.skew_sym_mat((omega_prev-b_omega_prev))
        F[3:6,12:15] = -self.Id3
        F[6:9,3:6] = self.Id3
        F[6:9,6:9] = -self.skew_sym_mat((omega_prev-b_omega_prev))
        
        # AD_x[:6,:6] = torch.eye(6)
        # AD_x[9:,6:] = torch.eye(12)
        F = F * dt
        F_square = F.mm(F)
        F_cube = F.mm(F).mm(F)
        Phi = self.IdP + F + 1/2*F_square + 1/6*F_cube
        P = Phi.mm(P_prev + Q).mm(Phi.t())
        return X_propag,b_omega_propag,b_acc_propag,R_c_propag,p_c_propag,   P
                
    def state_covariance_update(self,X_t,b_omega_t, b_acc_t, R_c_t, p_c_t , P_t,N_n_c, p_GPS):
        
        
        """"
        parameters:
        X_t: state matrix of IMU w.r.t the world frame [R v p; 0 0 0 1 0; 0 0 0 0 0 1]
        b_omega_t: IMU omega bias
        b_acc_t: IMU acc bias
        R_c_t: Rotation matrix from IMU frame to C frame, or C w.r.t IMU, in the IMU frame.
        p_c_t: p vector from C frame to IMU frame, in the IMU frame
        P_t: process error matrix
        p_GPS: GPS system measured position
        N_n_c: measurement covariance matrix in the car frame, should be a diag matrix
        """
        
        
        
        #update state based on propagation
        #these variables are with respect to world frame
        Rot = X_t[:3,:3].clone()
        v = X_t[:3,3]
        p = X_t[:3,4]
        p_c_world = p-Rot.mv(p_c_t)

        Rot_c = Rot.mm(R_c_t)

        #update H_t
        H_t = torch.zeros([3,21])
        H_t[:,:3] = self.skew_sym_mat(p_c_t)
        H_t[:,6:9] = torch.eye(3).clone()
        #H_t[:,15:18] = self.skew_sym_mat(p_c_t)
        
        #update measure and estimate error z_t
        z_t_c = p_GPS - (p_c_world)
        z_t = Rot.t().mv(z_t_c)
        #print('z_t',z_t)
        #update N_n
        N_n = Rot_c.t().mm(N_n_c).mm(Rot_c)
        #N_n = N_n_c
        
        
        C=H_t.mm(P_t).mm(H_t.t())+N_n
        B=P_t.mm(H_t.t())
        #K_t=torch.linalg.solve(C,B,left=False)
        K_t = (torch.linalg.solve(C, (B).t())).t()
        #print(K_t)
        depsilon = K_t.mv(z_t)
        #print(depsilon)
        dX = self.se2_3alg2gro(depsilon[:9])
        
        X_up = X_t.mm(dX)
         #update bias_angular and bias_acc
        b_omega_up = b_omega_t + depsilon[9:12]
        #print(b_omega_up)
        b_acc_up = b_acc_t + depsilon[12:15]
        
        #update epsilon_R_c_t = log(R_c_up)
        R_c_up = self.so3_alg2gro(depsilon[15:18]).mm(R_c_t)
        
        #update p_c
        p_c_up = p_c_t + depsilon[18:21]
        
        
        
        #update error
        KH = K_t.mm(H_t)
        I_KH = torch.eye(KH.size()[0])-KH
        P_up = I_KH.mm(P_t).mm(I_KH.t())+K_t.mm(N_n).mm(K_t.t())
        P_up = (P_up + P_up.t())/2
        
        return X_up,b_omega_up,b_acc_up,R_c_up,p_c_up,  P_up, z_t,p_c_world
        
        

    def Rot2phi(self,Rot):
        Cha_mat = Rot-torch.eye(3)
        Cha_col1 = Cha_mat[0,:]
        Cha_col2 = Cha_mat[1,:]
        Cha_col3 = Cha_mat[2,:]
        
        axis_dir1 = torch.linalg.cross(Cha_col1,Cha_col2)
        axis_dir2 = torch.linalg.cross(Cha_col1,Cha_col2)
        axis_dir3 = torch.linalg.cross(Cha_col2,Cha_col3)
        
        axis_dir=axis_dir1
        if (axis_dir.equal(torch.zeros([1,3]))):
                axis_dir=axis_dir2
                if(axis_dir.equal(torch.zeros([1,3]))):
                        axis_dir=axis_dir3
        
        
        axis = axis_dir/torch.linalg.norm(axis_dir)
        angle = torch.acos((torch.trace(Rot)-1)/2)
        return angle, axis
    
    
    
    def se2_3alg2gro(self, xi):
        phi = xi[:3]
        angle = torch.linalg.norm(phi)
        # Near |phi|==0, use first order Taylor expansion
        if torch.abs(angle) < 1e-8:
                skew_phi = self.skew_sym_mat(phi)
                J = self.Id3 + 0.5 * skew_phi
                Rot = self.Id3 + skew_phi
        else:
                axis = phi / angle
                skew_axis = self.skew_sym_mat(axis)
                s = torch.sin(angle)
                c = torch.cos(angle)
                J = (s/angle) * self.Id3 + (1-s/angle) * torch.outer(axis,axis) + ((1-c)/angle) * skew_axis
                Rot = c * self.Id3 + (1 - c) * torch.outer(axis, axis) + s * skew_axis
        x = J.mm(xi[3:].view(-1, 3).t())
        dX = self.se2_3_state_mat(Rot,x[:,0],x[:,1])
        return dX

    def so3_alg2gro(self, phi):
            angle = torch.linalg.norm(phi)
            # Near phi==0, use first order Taylor expansion
            if torch.abs(angle) < 1e-8:
                    skew_phi = self.skew_sym_mat(phi)
                    return self.Id3 + skew_phi
            else:
                    axis = phi / angle
                    skew_axis = self.skew_sym_mat(axis)
                    s = torch.sin(angle)
                    c = torch.cos(angle)
                    Rot = c * self.Id3 + (1 - c) * torch.outer(axis, axis) + s * skew_axis
                    return Rot
    
    def skew_sym_mat(self,x):
            X = torch.Tensor([[0, -x[2], x[1]],
                            [x[2], 0, -x[0]],
                            [-x[1], x[0], 0]])
            return X
                                                    
    def se2_3_state_mat(self,Rot,v,p):
            X=torch.zeros([5,5])
            X[:3,:3] = Rot
            X[:3,3] = v
            X[:3,4] = p
            X[3,3] = 1
            X[4,4] = 1
            return X
    
    def normalize_rot(self,rot):
        # U, S, V = torch.svd(A) returns the singular value
        # Irrespective of the original strides, the returned matrix U will
        # be transposed, i.e. with strides (1, n) instead of (n, 1).

        # pytorch SVD seems to be inaccurate, so just move to numpy immediately
        U, _, V = torch.svd(rot)
        S = torch.eye(3)
        S[2, 2] = torch.det(U) * torch.det(V)
        return U.mm(S).mm(V.t())
    
if __name__ == '__main__':
        rospy.init_node('LIEKF')
        rospy.loginfo('helloworld')
        iekf_filter = LIEKF()
        rospy.Subscriber('/init_proc',String,iekf_filter.init_protocol_callback)
        rospy.Subscriber('/sync/quaternion',QuaternionStamped,iekf_filter.vicon_quaternion_callback)
        
        while iekf_filter.imu_cal_flag == 0:
            pass
        rospy.Subscriber('/sync/gather',sync_gather,iekf_filter.vicon_run_callback)
        #rospy.Subscriber('/sync/imu',Imu,iekf_filter.IMU_run_callback)
        iekf_filter.run()

        

        
