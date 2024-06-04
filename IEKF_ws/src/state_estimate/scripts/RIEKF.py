import torch
import rospy
from sensor_msgs.msg import Imu
from std_msgs.msg import String
from geometry_msgs.msg import TransformStamped
from scipy.spatial.transform import Rotation as Rotlib
from state_estimate.msg import state_IMU
from time_synchronization.msg import sync_gather
import numpy as np


#function definition
def Quat2Rot(Quaternion):
    q0 = Quaternion[0]
    q1 = Quaternion[1]
    q2 = Quaternion[2]
    q3 = Quaternion[3]
    #print(torch.tensor([1-2*(q2*q2+q3*q3), 2*(q1*q2-q0*q3)]))
    R = torch.tensor([[1-2*(q2*q2+q3*q3),  2*(q1*q2-q0*q3),  2*(q0*q2+q1*q3)],
                      [2*(q1*q2+q0*q3),    1-2*(q1*q1+q3*q3),2*(q2*q3-q0*q1)],
                      [2*(q1*q3-q0*q2),    2*(q2*q3+q0*q1),   1-2*(q1*q1+q2*q2)]])
    return R

#IEKF class definition
class RIEKF:
    #use IMU data as input, and use velocity as measurement
     #parameters
        g = torch.tensor([0,0,-9.80665])
        P_dim = 21
        Q_dim = 18
        N_dim = 3
        Id2 = torch.eye(2)
        Id3 = torch.eye(3)
        Id6 = torch.eye(6)
        IdP = torch.eye(21)
        Id12 = torch.eye(12)
        Q = torch.zeros([18,18])
        N_n_c = torch.zeros([3,3])
        P = torch.zeros([21,21])
        z_std = torch.zeros([3])
        
        
        cov_omega = 1e-3                # gyro covariance
        cov_acc = 1e-2                  # accelerometer covariance
        cov_b_omega = 6e-9              # gyro bias covariance
        cov_b_acc = 2e-4                # accelerometer bias covariance
        cov_Rot_c_i = 1e-9              # car to IMU orientation covariance5
        cov_t_c_i = 1e-9                # car to IMU translation covariance

        # encoder-velocity-measurment covariance
        var_for = 0.1
        beta_for = 3
        var_lat = 0.1                  # Zero lateral velocity variance
        beta_lat = 3                    # scale factor var_lat for covariance
        var_up = 0.1                   # Zero upward velocity variance
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
        n_normalize_rot_c_i = 10000      # timestamp before normalizing car to IMU orientation


    #functions
        def __init__(self):
                super(RIEKF, self).__init__()
                self.Q = torch.diag(torch.Tensor([self.cov_omega,       self.cov_omega,     self. cov_omega,         # Set the state noise matix
                                          self.cov_acc,         self.cov_acc,       self.cov_acc,
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
                
                #self.N_n_c = torch.diag(torch.tensor([50.0,50.0,50.0]))
                self.N_n_c = torch.diag(torch.tensor([self.var_for,self.var_lat,self.var_up]))
                
        
        def run(self, time, u, z_covs, v_mes0, ang0,v_backwheel):
                """
                Run IEKF algorithm on input sequence
                :param t: time vector
                :param u: input measurement, u = [ax, ay, az, wx, wy, wz]
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

                                                                                                            # Set the delta time vector, that contain the delta of time for each sample
                N = u.shape[0]-2                                                                                                # Sequence length
                
                X = torch.zeros((N,5,5))
                Rot = torch.zeros((N, 3, 3))                                # Rotation matrix (orientation) in IMU-frame
                v = torch.zeros((N, 3))                                     # Velocity vector
                p = torch.zeros((N, 3))                                     # Position vector
                b_omega = torch.zeros((N, 3))                               # Angular speed biase
                b_acc = torch.zeros((N, 3))                                 # Acceleration biase
                R_c = torch.zeros((N, 3, 3))                            # Rotation matrix from car-frame to IMU-frame
                p_c = torch.zeros((N, 3))                                 # Translation vector from car-frame to IMU-frame
                z = torch.zeros([N,3])
                rot0 = Rotlib.from_euler('xyz', [ang0[0], ang0[1], ang0[2]])
                Rot[0,:,:] = torch.from_numpy(rot0.as_matrix())                 # Set initial car orientation
                Rot[0,:,:] = vicon_rotation_arr[:,:,0]
                print(Rot[0,:,:])
                v[0,:] = v_mes0                                               # Set initial velocity vector
                R_c[0] = torch.eye(3)                                   # Set initial rotation between car and IMU frame to 0, i.e. identity matrix  
                p_c[0] = torch.tensor([390e-3,-340e-3,841.375e-3])
                p[0] = torch.tensor([-0.8183,1.5220,0.9433])+p_c[0]
                b_omega[0] = torch.tensor([torch.mean(u[:300,0]),torch.mean(u[:300,1]),torch.mean(u[:300,2])])
                b_acc[0] = torch.tensor([torch.mean(u[:300,3]), torch.mean(u[:300,4]) ,0])
                b_acc[0,2] = torch.mean(u[:300,5])-9.80665
                
                X[0,:3,:3] = Rot[0]
                X[0,:3,3] = v[0,:].clone()
                X[0,:3,4] = p[0,:].clone()
                X[0,3,3] = 1
                X[0,4,4] = 1
                P_store[0,:,:] = self.P.clone()
                
                # Initialise the states variables with initial condictions
                #measurements_covs = self.z_to_cov(z_covs)

                
                #run IEKF algorithm
                # first propagate based on last-time update  (predict)
                # then update
                for i in range(1, N):
                        print(i)
                        dt = time[i]-time[i-1] 
                        X_propag,b_omega_propag,b_acc_propag,R_c_propag,p_c_propag,   P_propag = \
                                self.propagate(P_store[i-1],u[i][:3],u[i][3:],X[i-1],dt,b_omega[i-1],b_acc[i-1],R_c[i-1],p_c[i-1])
                        X[i], b_omega[i], b_acc[i], R_c[i], p_c[i], P_store[i,:,:],z[i,:] = \
                                self.state_covariance_update(X_propag,u[i][:3],u[i][3:],b_omega_propag,b_acc_propag,R_c_propag,p_c_propag,P_propag,self.N_n_c,v_backwheel[0,i])
                                                        #self,X_t, b_omega_t, b_acc_t, R_c_t, p_c_t , P_t, V_t, N_n_c,v_1
                        #X[i], b_omega[i], b_acc[i], R_c[i], p_c[i], P_store[i,:,:],z[i,:],z_gt[i,:]= \
                                #self.state_covariance_update(X_propag,b_omega_propag,b_acc_propag,R_c_propag,p_c_propag,P_propag,self.N_n_c,p_GPS[:,i],p_gt[:,i])
                        p[i,:] = X[i][:3,4]
                        # correct numerical error every second
                        if i % self.n_normalize_rot == 0:
                                Rot[i] = self.normalize_rot(Rot[i])
                        # correct numerical error every 10 seconds
                        if i % self.n_normalize_rot_c_i == 0:
                                R_c[i] = self.normalize_rot(R_c[i])
                                
                plt.figure(num=6)
                ax = plt.axes(projection = '3d')
                plt.plot(p[:,0],p[:,1],p[:,2])
                plt.axis('equal')
                
                plt.figure(num=7)
                plt.plot(z[:,0],label='x')
                plt.plot(z[:,1],label='y')
                plt.plot(z[:,2],label='z')
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
                
        def propagate(self,P_prev,omega_prev,acc_prev,X_prev,dt,b_omega_prev,b_acc_prev,R_c_prev,p_c_prev):
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
                
                #
                # print(X_propag)
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
                F[:3, 9:12] = -Rot_prev
                F[3:6, :3] = self.skew_sym_mat(self.g)
                F[3:6, 9:12] = -v_skew_rot
                F[3:6, 12:15] = -Rot_prev
                F[6:9, 3:6] = self.Id3 
                F[6:9, 9:12] = -p_skew_rot
                
                #fill AD_x matrix
                AD_x[:3, :3] = Rot_prev
                AD_x[3:6, :3] = v_skew_rot
                AD_x[3:6, 3:6] = Rot_prev
                AD_x[6:9, :3] = p_skew_rot
                AD_x[6:9,6:9] = Rot_prev
                AD_x[9:15, 6:12] = self.Id6
                AD_x[15:18, 12:15] = self.Id3
                AD_x[18:21, 15:18] = self.Id3
                #AD_x[9:21,9:21] = self.Id12
                
                F = F * dt
                AD_x = AD_x * dt
                F_square = F.mm(F)
                F_cube = F.mm(F).mm(F)
                #taylor expansion of exp(Adt), take first 4 terms
                Phi = self.IdP + F + 1/2*F_square + 1/6*F_cube
                P = Phi.mm(P_prev + AD_x.mm(Q).mm(AD_x.t())).mm(Phi.t())
                return X_propag,b_omega_propag,b_acc_propag,R_c_propag,p_c_propag,   P
                
                
        def state_covariance_update(self,X_t,omega_imu,acc_imu, b_omega_t, b_acc_t, R_c_t, p_c_t , P_t, N_n_c,v_1):
                """"
                parameters:
                X_t: state matrix of IMU w.r.t the world frame [R v p; 0 0 0 1 0; 0 0 0 0 0 1]
                b_omega_t: IMU omega bias
                b_acc_t: IMU acc bias
                R_c_t: Rotation matrix from IMU frame to C frame, or C w.r.t IMU, in the IMU frame.
                p_c_t: p vector from C frame to IMU frame, in the IMU frame
                P_t: process error matrix
                v_1: encoder velocity v1
                v_2: encoder velocity v2
                N_n_c: measurement covariance matrix in the car frame, should be a diag matrix
                """
                
                #update state based on propagation
                #these variables are with respect to world frame
                Rot = X_t[:3,:3].clone()
                v = X_t[:3,3]
                p = X_t[:3,4]
                
                omega = omega_imu-b_omega_t
                acc = acc_imu-b_acc_t
                Rot_c = Rot.mm(R_c_t)
                v_c_IMU = Rot.t().mv(v)+self.skew_sym_mat(omega).mv(p_c_t)
                v_c = R_c_t.t().mv(v_c_IMU)
                print("p_car_imu",p_c_t)
                #calculate observe error
                z_t_c = torch.tensor([v_1,0,0])-v_c
                #print(v_1)
                z_t = Rot_c.mv(z_t_c)
                print("error: ",z_t)
                
                
                #update H_t
                A = R_c_t.t()
                H_t = torch.zeros([3,21])
                H_t[:,3:6] = A
                #H_t[:,9:12] = -A.mm(self.skew_sym_mat(p_c_t))
                H_t[:,15:18] = A.mm(Rot.t()).mm(self.skew_sym_mat(v)).mm(Rot)
                H_t[:,18:21] = -A.mm(self.skew_sym_mat(omega))
                
                #update N_n
                N_n = Rot_c.mm(N_n_c).mm(Rot_c.t())
                
                
                #calculate Kalman filter K_k
                #K=PHt(HPHt+V)^-1
                #then K(HPHt+V)=PHt
                
                C=H_t.mm(P_t).mm(H_t.t())+N_n
                B=P_t.mm(H_t.t())
                K_t=torch.linalg.solve(C,B,left=False)
                #K_t.size = 21,3
                
                #calculate depsilon in Lie Algebra se2(3)
                depsilon=K_t.mv(z_t)
                #print(depsilon)
                #transfer depsilon to Lie group SE2(3)
                dX = self.se2_3alg2gro(depsilon[:9])
                #update state (in Lie group)
                X_up = dX.mm(X_t)
                
                #update bias_angular and bias_acc
                b_omega_up = b_omega_t + depsilon[9:12]
                b_acc_up = b_acc_t + depsilon[12:15]
                print('b_acc',b_acc_up)
                #update epsilon_R_c_t = log(R_c_up)
                R_c_up = self.so3_alg2gro(depsilon[15:18]).mm(R_c_t)
                
                #update p_c
                p_c_up = p_c_t + depsilon[18:21]
                
                
                
                #update covariance of error epsilon
                KH = K_t.mm(H_t)
                I_KH = torch.eye(KH.size()[0])-KH
                P_up = I_KH.mm(P_t).mm(I_KH.t())+K_t.mm(N_n).mm(K_t.t())
                P_up = (P_up + P_up.t())/2
                return X_up,b_omega_up,b_acc_up,R_c_up,p_c_up,  P_up,z_t
            
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
            return angle*axis
        
        
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
                        return c * self.Id3 + (1 - c) * torch.outer(axis, axis) + s * skew_axis
        
        
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

        iekf_filter = RIEKF()

        import matplotlib.pyplot as plt
        import pandas as pd
        
        import matplotlib as mpl
        print(mpl.get_backend())
        mpl.use('QtAgg')

        import rosbag
        import rospy
        import matplotlib.pyplot as plt
        import os

        os.environ["KMP_DUPLICATE_LIB_OK"] = "TRUE"

        bag = rosbag.Bag('2023-1-31/2023-01-31-19-11-13.bag')
        print(bag)
        vicon_msgStructs = bag.read_messages(topics='/sync/vicon')
        bag_start = bag.get_start_time()
        bag_end = bag.get_end_time()

        #parameters
        n_imu_msg = 17839
        n_vicon_msg = n_imu_msg

        #vicon data reading
        vicon_time_arr = torch.zeros([n_vicon_msg])
        vicon_data_arr = torch.zeros([3,n_vicon_msg]) #observe measurement
        vicon_rotation_arr = torch.zeros([3,3,n_vicon_msg])
        index = 0
        for topic, msg, t in vicon_msgStructs:
                header = msg.header
                vicon_data_arr[0,index] = msg.transform.translation.x
                vicon_data_arr[1,index] = msg.transform.translation.y
                vicon_data_arr[2,index] = msg.transform.translation.z
                vicon_time_arr[index] = torch.tensor(t.to_sec()-bag_start)
                vicon_rotation_arr[:,:,index] = Quat2Rot(torch.tensor([msg.transform.rotation.x,msg.transform.rotation.y,msg.transform.rotation.z,msg.transform.rotation.w]))
                index += 1
        #print(vicon_x)


        print(vicon_time_arr)

        imu_msgStructs =  bag.read_messages(topics='/sync/imu')

        #imu data reading
        imu_time_arr = torch.zeros([n_imu_msg])
        u = torch.zeros([6,n_imu_msg]) #u: inputs of state equation
        index = 0

        for topic, msg, t in imu_msgStructs:
                header = msg.header
                u[0,index] = msg.angular_velocity.x
                u[1,index] = msg.angular_velocity.y
                u[2,index] = msg.angular_velocity.z
                
                u[3,index] = msg.linear_acceleration.x
                u[4,index] = msg.linear_acceleration.y
                u[5,index] = msg.linear_acceleration.z
                imu_time_arr[index] = torch.tensor(t.to_sec()-bag_start)
                index += 1
        
        #encoder data reading

        encoder_msgStructs =  bag.read_messages(topics='/sync/encoder')
        encoder_time_arr = torch.zeros([n_imu_msg])

        v_backwheel = torch.zeros([2,n_imu_msg]) #u: inputs of state equation
        index = 0

        for topic, msg, t in encoder_msgStructs:
                header = msg.header
                v_backwheel[0,index] = msg.encoder3
                # if (v_backwheel[0,index]-v_backwheel[0,index-1]).abs()>10:
                #         v_backwheel[0,index] = v_backwheel[0,index-1]
                v_backwheel[0,index] *= -1/2000*(3.1415926*45.3e-3)*508/390*100               
                v_backwheel[1,index] = -msg.encoder4/2000*(3.1415926*45.3e-3)*508/400*100
                encoder_time_arr[index] = torch.tensor(t.to_sec()-bag_start)
                index += 1
            
        print(imu_time_arr)

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
        plt.axis('equal')


        plt.figure(num=4)
        plt.plot(imu_time_arr,label='imu')
        plt.plot(vicon_time_arr,label='vicon')
        plt.legend()
        
        plt.figure(num=5)
        plt.plot(imu_time_arr.cpu().numpy(),v_backwheel[0,:])
        


        # Export the values to an np.array
        #v_mes0 = dataset[["ve", "vn", "vu"]].copy().iloc[0, :].values
        #ang0 = dataset[["roll", "pitch", "yaw"]].copy().iloc[0, :].values
        
        v_mes0 = torch.tensor([0,0,0])
        ang0 = torch.tensor([0,0,0])

        print(f"Initial conditions:\n\tvelocity: {v_mes0}\n\torientation: {ang0}")

        kalman_1 = iekf_filter.run(imu_time_arr, u, np.ones((imu_time_arr.shape[0], 1))@[[1., -0.5]], v_mes0, ang0,v_backwheel)
        
        plt.figure()
        plt.plot(kalman_1['p'][:, 0], kalman_1['p'][:, 1], 'g-.')
        
        #kalman_2 = iekf_filter.run(t, u, np.ones((t.shape[0], 1))@[[0, 0]], v_mes0, ang0)
       # plt.plot(kalman_2['p'][:, 0], kalman_2['p'][:, 1], 'g-.')

        plt.axis('equal')
        
        

        print(f"\n#####\nProgram run time: {round(time.time()-start_time, 1)} s\n#####")

        plt.show()