import torch
import time
import numpy as np
from scipy.spatial.transform import Rotation as Rotlib
import random
import pandas as pd
import matplotlib.pyplot as plt
np.set_printoptions(precision=2)

#function definition
def Quat2Rot(Quaternion):
    q0 = Quaternion[3]
    q1 = Quaternion[0]
    q2 = Quaternion[1]
    q3 = Quaternion[2]
    #print(torch.tensor([1-2*(q2*q2+q3*q3), 2*(q1*q2-q0*q3)]))
    R = torch.tensor([[1-2*(q2*q2+q3*q3),  2*(q1*q2-q0*q3),  2*(q0*q2+q1*q3)],
                      [2*(q1*q2+q0*q3),    1-2*(q1*q1+q3*q3),2*(q2*q3-q0*q1)],
                      [2*(q1*q3-q0*q2),    2*(q2*q3+q0*q1),   1-2*(q1*q1+q2*q2)]])
    return R

#IEKF class definition
class LIEKF:
    #parameters
    
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


    #functions
    def __init__(self) -> None:
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
               
    def run(self, time, u, z_covs, v_mes0, ang0,p_GPS,p_gt):
        """
        Run IEKF algorithm on input sequence
        :param t: time vector
        :param u: input measurement, u = [ wx, wy, wz,ax, ay, az]
        :param z_covs: pseudo-measure covariance, [cov_v_lat, cov_v_up]
        :param v_0: initial velocity
        :param ang0: initial orientation
        :return:
        """
        #initialization
        u = torch.Tensor(u).cpu() if type(u).__module__ == np.__name__ else u.cpu()
        z_covs = torch.Tensor(z_covs).cpu() if type(z_covs).__module__ == np.__name__ else z_covs.cpu()
        v_mes0 = torch.Tensor(v_mes0).cpu() if type(v_mes0).__module__ == np.__name__ else v_mes0.cpu()
        u = u.t()
        P_store = torch.zeros((time.shape[0], 21, 21))
        P_store_imu = torch.zeros((time.shape[0], 21, 21))
        
        #dt = t[1:] - t[:-1]                                                                                             # Set the delta time vector, that contain the delta of time for each sample
        N = u.shape[0]-10                                                                                                  # Sequence length
        
        X = torch.zeros((N,5,5))
        Rot = torch.zeros((N, 3, 3))                                # Rotation matrix (orientation) in car-frame
        v = torch.zeros((N, 3))                                     # Velocity vector
        p = torch.zeros((N, 3))                                     # Position vector
        p_world = torch.zeros((N,3))
        b_omega = torch.zeros((N, 3))                               # Angular speed biase
        b_acc = torch.zeros((N, 3))                                 # Acceleration biase
        R_c = torch.zeros((N, 3, 3))                            # Rotation matrix from car-frame to IMU-frame
        p_c = torch.zeros((N, 3))                                 # Translation vector from car-frame to IMU-frame
        z = torch.zeros([N,3])
        z_gt = torch.zeros([N,3])
        rot0 = Rotlib.from_euler('xyz', [ang0[0], ang0[1], ang0[2]])
        Rot[0,:,:] = torch.from_numpy(rot0.as_matrix())                 # Set initial car orientation
        Rot[0,:,:] = GPS_rotation[:,:,0]
        #Rot[0,:,:] = vicon_rotation_arr[:,:,4020]
        print(Rot[0,:,:])
        v[0,:] = v_mes0                                               # Set initial velocity vector
        R_c[0] = torch.eye(3)                                   # Set initial rotation between car and IMU frame to 0, i.e. identity matrix  
        p_c[0] = torch.tensor([-117.380811e-3,-517.9640275e-3,-27.2120583e-3])
        p[0] = Rot[0].mv(p_c[0])
        b_omega[0] = torch.tensor([0.001,-0.001,0.002])
        b_acc[0] = torch.tensor([torch.mean(u[:300,3]), torch.mean(u[:300,4]) ,0])
        b_acc[0,2] = torch.mean(u[:300,5])-9.80665
        
        X[0,:3,:3] = Rot[0]
        X[0,:3,3] = v[0,:].clone()
        X[0,:3,4] = p[0,:].clone()
        X[0,3,3] = 1
        X[0,4,4] = 1
        P_store[0,:,:] = self.P.clone()
        
        X_imu = torch.zeros((N,5,5))
        Rot_imu = torch.zeros((N, 3, 3))                                # Rotation matrix (orientation) in car-frame
        v_imu = torch.zeros((N, 3))                                     # Velocity vector
        p_imu = torch.zeros((N, 3))                                     # Position vector
        b_omega_imu = torch.zeros((N, 3))                               # Angular speed biase
        b_acc_imu = torch.zeros((N, 3))                                 # Acceleration biase
        R_c_imu = torch.zeros((N, 3, 3))                            # Rotation matrix from car-frame to IMU-frame
        p_c_imu = torch.zeros((N, 3))                                 # Translation vector from car-frame to IMU-frame
        z_imu = torch.zeros([N,3])
        z_gt_imu = torch.zeros([N,3])
        rot0_imu = Rotlib.from_euler('xyz', [ang0[0], ang0[1], ang0[2]])
        Rot_imu[0,:,:] = torch.from_numpy(rot0.as_matrix())                 # Set initial car orientation
        Rot_imu[0,:,:] = vicon_rotation_arr[:,:,1]
        rot_euler = torch.zeros([N,3])
        print(Rot_imu[0,:,:])
        v_imu[0,:] = v_mes0                                               # Set initial velocity vector
        R_c_imu[0] = torch.eye(3)                                   # Set initial rotation between car and IMU frame to 0, i.e. identity matrix  
        p_c_imu[0] = torch.tensor([-0.110-0.006253,(0.102+0.01175),(0.081-0.007645)])
        p_imu[0] = torch.tensor([-1.5472,1.4863,0.9659])+p_c[0]
        b_omega_imu[0] = torch.tensor([0.001,-0.001,0.002])
        b_acc_imu[0] = torch.tensor([torch.mean(u[:300,3]), torch.mean(u[:300,4]) ,0])
        b_acc_imu[0,2] = torch.mean(u[:300,5])-9.80665
        
        X_imu[0,:3,:3] = Rot_imu[0]
        X_imu[0,:3,3] = v_imu[0,:].clone()
        X_imu[0,:3,4] = p_imu[0,:].clone()
        X_imu[0,3,3] = 1
        X_imu[0,4,4] = 1
        P_store_imu[0,:,:] = self.P.clone()
        # Initialise the states variables with initial condictions
       # measurements_covs = self.z_to_cov(z_covs)
        print('R:',self.so3_alg2gro(torch.tensor([0.0,2.0,1.0])/torch.tensor([5]).sqrt()*torch.tensor([3.1415926/6])))
        R_temp = self.so3_alg2gro(torch.tensor([0.0,2.0,1.0])/torch.tensor([5]).sqrt()*torch.tensor([3.1415926/6]))
        a = torch.tensor([3.0,0,0])
        print(R_temp.mv(a))
        #run IEKF algorithm
        # first propagate based on last-time update  (predict)
        # then update
        for i in range(1, N):
            #print(i)
            dt = time[i]-time[i-1]
            
            R = torch.tensor([  [0,-1,0],
                              [0,0,1],
                              [-1,0,0]])
            #print(self.Rot2phi(R))time
            X_propag,b_omega_propag,b_acc_propag,R_c_propag,p_c_propag,   P_propag = \
                self.propagate(P_store[i-1],u[i][:3],u[i][3:],X[i-1],dt,b_omega[i-1],b_acc[i-1],R_c[i-1],p_c[i-1])
                #propagate(self,P_prev,omega_prev,acc_prev,X_prev,dt,b_omega_prev,b_acc_prev,R_c_prev,p_c_prev):
            X_imu[i],b_omega_imu[i],b_acc_imu[i],R_c_imu[i],p_c_imu[i],P_store_imu[i,:,:] = \
                self.propagate(P_store_imu[i-1],u[i][:3],u[i][3:],X_imu[i-1],dt,b_omega_imu[i-1],b_acc_imu[i-1],R_c_imu[i-1],p_c_imu[i-1])
                
            X[i], b_omega[i], b_acc[i], R_c[i], p_c[i], P_store[i,:,:],z[i,:],z_gt[i,:],p_world[i,:]= \
                self.state_covariance_update(X_propag,b_omega_propag,b_acc_propag,R_c_propag,p_c_propag,P_propag,self.N_n_c,p_GPS[:,i],p_gt[:,i])
            #print(p_GPS[:,i])
            self.z_std = self.z_std + torch.abs(z_gt[i,:])
            #print('z_std')
            #print(self.z_std)
            #print('z_std')
            Rot[i,:,:] = X[i][:3,:3]
            rot = Rotlib.from_matrix(Rot[i,:,:])
            rot_euler[i,:] = torch.tensor([rot.as_euler('zxy',degrees=True)])
            print('Rot',Rot[i,:,:])
            print('rot:', rot.as_euler('zxy',degrees=True))
            p[i,:] = X[i][:3,4]
            p_imu[i,:] = X_imu[i][:3,4]
            
            v[i,:] = X[i][:3,3]
            print('v:',v[i,:])
            #print(p_imu[i,:])
            # correct numerical error every second
            if i % self.n_normalize_rot == 0:
                Rot[i] = self.normalize_rot(Rot[i])
            # correct numerical error every 10 seconds
            if i % self.n_normalize_rot_c_i == 0:
                R_c[i] = self.normalize_rot(R_c[i])

        plt.figure(num=6)
        #ax = plt.axes(projection = '3d')
        plt.plot(p_world[:,0],p_world[:,1],label='estimate')
        plt.plot(vicon_data_arr[0,:],vicon_data_arr[1,:],label='RTK')
        plt.legend()
        plt.axis('equal')
        
        plt.figure(num=7)
        plt.plot(z_gt[:,0],label='x')
        plt.plot(z_gt[:,1],label='y')
        plt.plot(z_gt[:,2],label='z')
        plt.legend()
        
        plt.figure(num=8)
        ax = plt.axes(projection = '3d')
        plt.plot(p_imu[:,0],p_imu[:,1],p_imu[:,2])
        #plt.axis('equal')
        
        plt.figure(num = 9)
        #ax = plt.axes(projection = '3d')
        plt.plot(p_c[:,0],label='x')
        plt.plot(p_c[:,1],label='y')
        plt.plot(p_c[:,2],label='z')
        plt.legend()
        
        plt.figure(num=10)
        
        plt.plot(v[:,0],label='x')
        plt.plot(v[:,1],label='y')
        plt.plot(v[:,2],label='z')
        plt.legend()

        plt.figure(num=11)
        plt.plot(np.arange(N),rot_euler[:,0],label='estimate')
        plt.plot(np.arange(N),GPS_euler[:N,2],label='GPS')
        plt.legend()
        
        
        Rot = Rot.numpy() if type(Rot).__module__ == torch.__name__ else Rot
        v = v.numpy() if type(v).__module__ == torch.__name__ else v
        p = p.numpy() if type(p).__module__ == torch.__name__ else p
        b_omega = b_omega.numpy() if type(b_omega).__module__ == torch.__name__ else b_omega
        b_acc = b_acc.numpy() if type(b_acc).__module__ == torch.__name__ else b_acc
        R_c = R_c.numpy() if type(R_c).__module__ == torch.__name__ else R_c
        p_c = p_c.numpy() if type(p_c).__module__ == torch.__name__ else p_c
        P_store = P_store.numpy() if type(P_store).__module__ == torch.__name__ else P_store
        return {"rot": Rot, "v": v, "p": p, "b_omega": b_omega, "b_acc": b_acc, "Rot_c_i": R_c, "t_c_i": p_c, "P_store": P_store}

        
        
    #functions
    def propagate(self,P_prev,omega_prev,acc_prev,X_prev,dt,b_omega_prev,b_acc_prev,R_c_prev,p_c_prev):
        #SE2(3) state matrix: [R_3,3  v_3,1 p_3,1
        #                      0_1,3  1     0
        #                      0_1,3  0     1     ]
        
        
        Propag_mat = torch.zeros([5,5])
        Rot_prev = X_prev[:3,:3].clone()
        v_prev = X_prev[:3,3].clone()
        p_prev = X_prev[:3,4].clone()
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
        
        #print(X_propag)
        #other state variable propagate (just clone)
        b_omega_propag = b_omega_prev
        b_acc_propag = b_acc_prev
        R_c_propag = R_c_prev.clone()
        p_c_propag = p_c_prev
        
        #covariance propagate
        F =  torch.zeros([self.P_dim, self.P_dim])
        AD_x = torch.zeros([self.P_dim,self.Q_dim])
        Q = self.Q.clone()
        
        v_skew_rot = self.skew_sym_mat(v_prev).mm(Rot_prev)
        p_skew_rot = self.skew_sym_mat(p_prev).mm(Rot_prev)
        
        #fill F matrix
        F[:3,:3] = -self.skew_sym_mat((omega_prev-b_omega_prev))
        #print(-self.Id3)
        F[:3,9:12] = -self.Id3
        F[3:6,:3] = -self.skew_sym_mat((acc_prev-b_acc_prev))
        F[3:6,3:6] = -self.skew_sym_mat((omega_prev-b_omega_prev))
        F[3:6,12:15] = -self.Id3
        F[6:9,3:6] = self.Id3
        F[6:9,6:9] = -self.skew_sym_mat((omega_prev-b_omega_prev))
        #print(F)
        
        # AD_x[:6,:6] = torch.eye(6)
        # AD_x[9:,6:] = torch.eye(12)
        F = F * dt
        F_square = F.mm(F)
        F_cube = F.mm(F).mm(F)
        Phi = self.IdP + F + 1/2*F_square + 1/6*F_cube
        P = Phi.mm(P_prev + Q).mm(Phi.t())
        #print(P)
        return X_propag,b_omega_propag,b_acc_propag,R_c_propag,p_c_propag,   P
                
    def state_covariance_update(self,X_t,b_omega_t, b_acc_t, R_c_t, p_c_t , P_t,N_n_c, p_GPS,p_gt):
        
        
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
        z_t_gt = p_gt - p_c_world
        print(z_t_gt)
        z_t = Rot.t().mv(z_t_c)
        #update N_n
        N_n = Rot_c.t().mm(N_n_c).mm(Rot_c)
        #N_n = N_n_c
        
        
        C=H_t.mm(P_t).mm(H_t.t())+N_n
        B=P_t.mm(H_t.t())
        K_t = (torch.linalg.solve(C, (B).t())).t()
        depsilon = K_t.mv(z_t)
        print('depsilon',depsilon)
        print(z_t)
        dX = self.se2_3alg2gro(depsilon[:9])
        
        X_up = X_t.mm(dX)
        #print(self.Rot2phi(X_up[:3,:3]))
         #update bias_angular and bias_acc
        b_omega_up = b_omega_t + depsilon[9:12]
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
        #print(X_up)
        
        return X_up,b_omega_up,b_acc_up,R_c_up,p_c_up,  P_up, z_t, z_t_gt,p_c_world
        
        

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
            
#     def z_to_cov(self, z):
#         cov = torch.zeros_like(z)
#         cov[:, 0] = self.var_lat**2 * 10**(self.beta_lat*z[:, 0])
#         cov[:, 1] = self.var_up**2 * 10**(self.beta_up*z[:, 1])
#         return cov
    
    def normalize_rot(self,rot):
        # U, S, V = torch.svd(A) returns the singular value
        # decomposition of a real matrix A of size (n x m) such that A=USV′.
        # Irrespective of the original strides, the returned matrix U will
        # be transposed, i.e. with strides (1, n) instead of (n, 1).

        # pytorch SVD seems to be inaccurate, so just move to numpy immediately
        U, _, V = torch.svd(rot)
        S = torch.eye(3)
        S[2, 2] = torch.det(U) * torch.det(V)
        return U.mm(S).mm(V.t())
    
if __name__ == '__main__':
        start_time = time.time()

        random_seed = 34                                                                                                    # set random seed
        rng = np.random.default_rng(random_seed)                                                                            # Create a RNG with a fixed seed
        random.seed(random_seed)                                                                                            # Set the Python seed

        iekf_filter = LIEKF()

        import matplotlib.pyplot as plt
        import pandas as pd
        
        import matplotlib as mpl
        print(mpl.get_backend())

        import rosbag
        import rospy
        import matplotlib.pyplot as plt
        import os

        os.environ["KMP_DUPLICATE_LIB_OK"] = "TRUE"


        import matplotlib as mpl
        print(mpl.get_backend())
        import rosbag
        import torch
        import rospy
        import matplotlib.pyplot as plt
        import os
        import math
        import numpy as np
        from geometry_msgs.msg import Transform
        from geometry_msgs.msg import Vector3
        import pymap3d as pm
        from scipy.spatial.transform import Rotation as Rotlib

        #function area

        def Quat2Rot(Quaternion):
            q0 = Quaternion[3]
            q1 = Quaternion[0]
            q2 = Quaternion[1]
            q3 = Quaternion[2]
            #print(torch.tensor([1-2*(q2*q2+q3*q3), 2*(q1*q2-q0*q3)]))
            R = torch.tensor([[1-2*(q2*q2+q3*q3),  2*(q1*q2-q0*q3),  2*(q0*q2+q1*q3)],
                            [2*(q1*q2+q0*q3),    1-2*(q1*q1+q3*q3),2*(q2*q3-q0*q1)],
                            [2*(q1*q3-q0*q2),    2*(q2*q3+q0*q1),   1-2*(q1*q1+q2*q2)]])
            return R


        os.environ["KMP_DUPLICATE_LIB_OK"] = "TRUE"

        bag = rosbag.Bag('2023-03-21-20-16-11.bag')  # stop-linear
        bag_start = bag.get_start_time()
        bag_end = bag.get_end_time()
        print(bag)
        run_msgStructs = bag.read_messages(topics='/sync/gather')

        n_sync = 1123


        """"
        Header header
        string child_frame_id
        geometry_msgs/Transform transform

        geometry_msgs/Vector3 angular_velocity
        geometry_msgs/Vector3 linear_acceleration

        int32 encoder3
        int16 encoder1

        """

        GPS_LLA = torch.zeros([n_sync,3],dtype=torch.float64)
        GPS_LLA0 = torch.zeros([3],dtype=torch.float64)
        GPS_ENU = torch.zeros([n_sync,3],dtype=torch.float32)
        GPS_quaternion = torch.zeros([n_sync,4],dtype=torch.float32)
        IMU_omega = torch.zeros([n_sync,3],dtype=torch.float32)
        IMU_acc = torch.zeros([n_sync,3],dtype=torch.float32)
        GPS_rotation = torch.zeros([3,3,n_sync],dtype=torch.float32)
        time_arr = torch.zeros([n_sync])
        index = 0
        GPS_euler = torch.zeros(n_sync,3)
        for topic, msg, t in run_msgStructs:
            GPS_LLA[index,0] = msg.transform.translation.x
            GPS_LLA[index,1] = msg.transform.translation.y
            GPS_LLA[index,2] = msg.transform.translation.z

            GPS_quaternion[index,0] = msg.transform.rotation.x
            GPS_quaternion[index,1] = msg.transform.rotation.y
            GPS_quaternion[index,2] = msg.transform.rotation.z
            GPS_quaternion[index,3] = msg.transform.rotation.w

            IMU_omega[index,0] = msg.angular_velocity.x
            IMU_omega[index,1] = msg.angular_velocity.y
            IMU_omega[index,2] = msg.angular_velocity.z

            IMU_acc[index,0] = msg.linear_acceleration.x
            IMU_acc[index,1] = msg.linear_acceleration.y
            IMU_acc[index,2] = msg.linear_acceleration.z

            quat_temp = Rotlib.from_quat(GPS_quaternion[index])
            print(torch.tensor(np.array(quat_temp.as_euler('xyz',degrees=True))))
            GPS_euler[index,:] = torch.tensor(np.array(quat_temp.as_euler('xyz',degrees=True)))
            GPS_rotation[:,:,index] = Quat2Rot(GPS_quaternion[index])
            time_arr[index] = torch.tensor(t.to_sec()-bag_start)
            index += 1
            GPS_LLA0 = GPS_LLA[0,:]


        for i in range(n_sync):
            tmp  = (pm.geodetic2enu(GPS_LLA[i,0],GPS_LLA[i,1],GPS_LLA[i,2],GPS_LLA0[0],GPS_LLA0[1],GPS_LLA0[2]))
            
            #print(tmp)
            GPS_ENU[i,0] = tmp[0]
            GPS_ENU[i,1] = tmp[1]
            GPS_ENU[i,2] = tmp[2]

        print(GPS_ENU.size())

        n_vicon_msg = n_sync
        vicon_time_arr = time_arr
        vicon_data_gt_arr = GPS_ENU.t()
        vicon_data_arr = GPS_ENU.t()
        vicon_rotation_arr = GPS_rotation

        n_imu_msg = n_sync
        print(vicon_time_arr)
        #print(vicon_rotation_arr[:,:,0]

        #imu data reading
        imu_time_arr = time_arr
        u = torch.zeros([6,n_imu_msg]) #u: inputs of state equation

        u[:3,:] = IMU_omega.t()
        u[3:6,:] = IMU_acc.t()

        fig = plt.figure(num=1)
        plt.plot(imu_time_arr.cpu().numpy(),u[0,:].cpu().numpy(),label='x')
        plt.plot(imu_time_arr.cpu().numpy(),u[1,:].cpu().numpy(),label='y')
        plt.plot(imu_time_arr.cpu().numpy(),u[2,:].cpu().numpy(),label='z')

        plt.figure(num=2)
        plt.plot(imu_time_arr,u[3,:],label='x')
        plt.plot(imu_time_arr,u[4,:],label='y')
        plt.plot(imu_time_arr,u[5,:],label='z')
        plt.legend()

        plt.figure(num=3)
        ax = plt.axes(projection = '3d')
        plt.plot(vicon_data_arr[0,:],vicon_data_arr[1,:],vicon_data_arr[2,:])


        plt.figure(num=4)
        plt.plot(imu_time_arr,label='imu')
        plt.plot(vicon_time_arr,label='vicon')
        plt.legend()
        
        


        # Export the values to an np.array
        #v_mes0 = dataset[["ve", "vn", "vu"]].copy().iloc[0, :].values
        #ang0 = dataset[["roll", "pitch", "yaw"]].copy().iloc[0, :].values
        
        v_mes0 = torch.tensor([0,0,0])
        ang0 = torch.tensor([0,0,(-120)/180*3.1415926])

        print(f"Initial conditions:\n\tvelocity: {v_mes0}\n\torientation: {ang0}")

        kalman_1 = iekf_filter.run(imu_time_arr, u, np.ones((imu_time_arr.shape[0], 1))@[[1., -0.5]], v_mes0, ang0,vicon_data_arr,vicon_data_gt_arr)
        
        plt.figure()
        plt.plot(kalman_1['p'][:, 0], kalman_1['p'][:, 1], 'g-.')
        
        #kalman_2 = iekf_filter.run(t, u, np.ones((t.shape[0], 1))@[[0, 0]], v_mes0, ang0)
       # plt.plot(kalman_2['p'][:, 0], kalman_2['p'][:, 1], 'g-.')

        plt.axis('equal')
        
        

        print(f"\n#####\nProgram run time: {round(time.time()-start_time, 1)} s\n#####")

        plt.show()
