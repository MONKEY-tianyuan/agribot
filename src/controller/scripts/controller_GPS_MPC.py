#!/usr/bin/env python3

import torch
import numpy
import time
import math
import rospy
import numpy as np
from sensor_msgs.msg import Imu
from std_msgs.msg import String
from sensor_msgs.msg import Image
from geometry_msgs.msg import TransformStamped
from geometry_msgs.msg import QuaternionStamped
from state_estimate.msg import state_IMU
from time_synchronization.msg import sync_gather
from serial_node.msg import ros_hub_vel,ros_step_pav,stm2ros_tao
from scipy.spatial.transform import Rotation as Rotlib
#from controller.msg import control_message
from state_estimate.msg import controll_message

pi = 3.1415926535

import matplotlib.pyplot as plt
import numpy as np
from math import *
from cvxopt import matrix, solvers
 
class MPC:
    def __init__(self):
        self.Np = 60  # Predict Length  
        self.Nc = 60  # Control Lenght
 
        self.dt = 0.01  # time step
        self.Length = 1.0  # vehicle verticle length
 
        max_steer = 60 * pi / 180  # maximum steering angle
        max_steer_v = 10 * pi / 180  # maximum steering velocity
        max_v = 2.5  # maximum velocity
        max_a = 0.5  #  maximum acceleration
 
        # loss function 
        self.Q = 50 * np.identity(3*self.Np)  # 位姿权重
        self.R = 100 * np.identity(2*self.Nc)  # 控制权重
 
        self.kesi = np.zeros((5, 1))
 
        self.A = np.identity(5)
 
        self.B = np.block([
            [np.zeros((3, 2))],
            [np.identity(2)]
        ])
 
        self.C = np.block([
            [np.identity(3), np.zeros((3, 2))]
        ])
 
        self.PHI = np.zeros((3*self.Np, 5))
        self.THETA = np.zeros((3*self.Np, 2*self.Nc))
 
        self.CA = (self.Np+1) * [self.C]
 
        self.H = np.zeros((2*self.Nc, 2*self.Nc))
 
        self.f = np.zeros((2*self.Nc, 1))
 
        # 不等式约束相关矩阵
        A_t = np.zeros((self.Nc, self.Nc))
        for p in range(self.Nc):
            for q in range(p+1):
                A_t[p][q] = 1
 
        A_I = np.kron(A_t, np.identity(2))
 
        # 控制量约束
        umin = np.array([[-max_v], [-max_steer]])
        umax = np.array([[max_v], [max_steer]])
        self.Umin = np.kron(np.ones((self.Nc, 1)), umin)
        self.Umax = np.kron(np.ones((self.Nc, 1)), umax)
 
        # 控制增量约束
        delta_umin = np.array([[-max_a * self.dt], [-max_steer_v * self.dt]])
        delta_umax = np.array([[max_a * self.dt], [max_steer_v * self.dt]])
        delta_Umin = np.kron(np.ones((self.Nc, 1)), delta_umin)
        delta_Umax = np.kron(np.ones((self.Nc, 1)), delta_umax)
 
        self.A_cons = np.zeros((2 * 2*self.Nc, 2*self.Nc))
        self.A_cons[0:2*self.Nc, 0:2*self.Nc] = A_I
        self.A_cons[2*self.Nc:4*self.Nc, 0:2*self.Nc] = np.identity(2*self.Nc)
 
        self.lb_cons = np.zeros((2 * 2*self.Nc, 1))
        self.lb_cons[2*self.Nc:4*self.Nc, 0:1] = delta_Umin
 
        self.ub_cons = np.zeros((2 * 2*self.Nc, 1))
        self.ub_cons[2*self.Nc:4*self.Nc, 0:1] = delta_Umax
 
    def mpcControl(self, x, y, yaw, v, angle, tar_x, tar_y, tar_yaw, tar_v, tar_angle):  # mpc优化控制
        T = self.dt
        L = self.Length
 
        # 更新误差
        self.kesi[0][0] = x-tar_x
        self.kesi[1][0] = y-tar_y
        self.kesi[2][0] = self.normalizeTheta(yaw - tar_yaw)
        self.kesi[3][0] = v - tar_v
        self.kesi[4][0] = angle - tar_angle
 
        # 更新A矩阵
        self.A[0][2] = -tar_v * sin(tar_yaw) * T
        self.A[0][3] = cos(tar_yaw) * T
        self.A[1][2] = tar_v * cos(tar_yaw) * T
        self.A[1][3] = sin(tar_yaw) * T
        self.A[2][3] = tan(tar_angle) * T / L
        self.A[2][4] = tar_v * T / (L * (cos(tar_angle)**2))
 
        # 更新B矩阵
        self.B[0][0] = cos(tar_yaw) * T
        self.B[1][0] = sin(tar_yaw) * T
        self.B[2][0] = tan(tar_angle) * T / L
        self.B[2][1] = tar_v * T / (L * (cos(tar_angle)**2))
 
        # 更新CA
        for i in range(1, self.Np+1):
            self.CA[i] = np.dot(self.CA[i-1], self.A)
 
        # 更新PHI和THETA
        for j in range(self.Np):
            self.PHI[3*j:3*(j+1), 0:5] = self.CA[j+1]
            for k in range(min(self.Nc, j+1)):
                self.THETA[3*j:3*(j+1), 2*k: 2*(k+1)
                           ] = np.dot(self.CA[j-k], self.B)
 
        # 更新H
        self.H = np.dot(np.dot(self.THETA.transpose(), self.Q),
                        self.THETA) + self.R
 
        # 更新f
        self.f = 2 * np.dot(np.dot(self.THETA.transpose(), self.Q),
                            np.dot(self.PHI, self.kesi))
 
        # 更新约束
        Ut = np.kron(np.ones((self.Nc, 1)), np.array([[v], [angle]]))
        self.lb_cons[0:2*self.Nc, 0:1] = self.Umin-Ut
        self.ub_cons[0:2*self.Nc, 0:1] = self.Umax-Ut
 
        # 求解QP
        P = matrix(self.H)
        q = matrix(self.f)
        G = matrix(np.block([
            [self.A_cons],
            [-self.A_cons]
        ]))
        h = matrix(np.block([
            [self.ub_cons],
            [-self.lb_cons]
        ]))
 
        solvers.options['show_progress'] = False
        sol = solvers.qp(P, q, G, h)
        X = sol['x']
 
        # 输出结果
        v += X[0]
        angle += X[1]
 
        return v, angle
 
    def normalizeTheta(self, angle):  # 角度归一化
        while(angle >= pi):
            angle -= 2*pi
 
        while(angle < -pi):
            angle += 2*pi
 
        return angle
 
    def findIdx(self, x, y, cx, cy):  # 寻找欧式距离最近的点
        min_dis = float('inf')
        idx = 0
 
        for i in range(len(cx)):
            dx = x - cx[i]
            dy = y - cy[i]
            dis = dx**2 + dy**2
            if(dis < min_dis):
                min_dis = dis
                idx = i
 
        return idx

    

class CONTROLLER:
    dist_hori = 0.8536 #unit: m
    L_vert = 1.2319
    L_diag = math.sqrt(dist_hori**2 + L_vert**2)
    p_c =  torch.tensor([-0.5588,0.2286,-0.91948],dtype=torch.double)
    
    psi = torch.zeros([3])
    pos = torch.zeros([3])
    vel = torch.zeros([3])

    omega = torch.zeros([3],dtype=torch.double)

    quat = torch.zeros([4])
    quat[3] = 1

    #state variables
    r_imu = torch.zeros([2])
    r_imu_dot = torch.zeros([2])
    r = torch.zeros([2])
    r_vicon = torch.zeros([2])
    r_abs_old = 0
    r_abs_new = 0
    r_dot = torch.zeros([2])
    psi_cur = torch.zeros([1])
    psi_0 = torch.zeros([1])
    psi_adj = torch.zeros([1])
    pos0 = torch.zeros([3])
    Rot0 = torch.zeros([3,3],dtype=torch.double)
    v_GPS = torch.zeros([1])

    encoder1 = 0
    encoder3 = 0

    v_back = 0
    v_back_init = 1.0
    v_front = 0
    encoder_step1 = 0
    encoder_step2 = 0

    Rot = torch.zeros([3,3],dtype=torch.double)
    dt = 0.01
    dt_std = 0.01
    t = 0
    t_init = 0.0
    run_flag = 0
    init_flag = 0

    #accleration counter
    acc_cnt = 400
    acc_index = 0
    run_index = 0
    over_index = 0
    idx = 0



    rd_num = 614
    rd_arr = np.zeros([2,614])
    rd_arr_adj = np.zeros([2,614])
    cx = np.zeros(rd_num)
    cy = np.zeros(rd_num)
    dx = np.zeros(rd_num)
    ddx = np.zeros(rd_num)
    cyaw = np.zeros(rd_num)
    ck = np.zeros(rd_num)



    pub_vel = rospy.Publisher('/cmd_hub_vel',ros_hub_vel,queue_size=10)
    pub_turn = rospy.Publisher('/cmd_step_pos',ros_step_pav,queue_size=10)
    pub_store = rospy.Publisher('/run_store',controll_message,queue_size=10)
    pub_img = rospy.Publisher('/camera_ctrl/image_color',Image,queue_size=10)

    mpc = MPC()

    def __init__(self):
        rospy.Subscriber('/run_motion',String,self.run_flag_callback)
        rospy.Subscriber('/state_estimate/state_estimate',state_IMU,self.state_estimate_callback)
        rospy.Subscriber('/encoder',stm2ros_tao,self.encoder_callback)
        rospy.Subscriber('/sync/vicon',TransformStamped,self.vicon_quaternion_callback)
        rospy.Subscriber('/filter/quaternion',QuaternionStamped,self.xsens_callback)
        rospy.Subscriber('/flir_camera_1/image_color',Image,self.image_callback)
        rospy.Subscriber('thermal_camera/image_raw',Image,self.thermal_image_callback)

        for i in range(self.rd_arr.shape[1]):
            if (i<100):
                self.rd_arr[1,i] = 0.0
                self.rd_arr[0,i] = 0.1*i
            elif (i>=100 and i<257):
                self.rd_arr[1,i] = -5.0 + 5.0*math.cos(0.02*(i-100))
                self.rd_arr[0,i] = 10.0 + 5.0*math.sin(0.02*(i-100))
            elif (i>=257 and i<357):
                self.rd_arr[1,i] = -10.0
                self.rd_arr[0,i] = 10.0 - 0.1*(i-257)
            elif (i>=357 and i<514):
                self.rd_arr[1,i] = -15.0 + 5.0*math.cos(0.02*(i-357))
                self.rd_arr[0,i] = -5.0*math.sin(0.02*(i-357))
            elif (i>=514):
                self.rd_arr[1,i] = -20.0
                self.rd_arr[0,i] = 0.1*(i-514)

        
    #callback functions
    def vicon_quaternion_callback(self,data):

        self.r_vicon[0] = data.transform.translation.x
        self.r_vicon[1] = data.transform.translation.y

    def xsens_callback(self,data):
        self.quat[0] = data.quaternion.x
        self.quat[1] = data.quaternion.y
        self.quat[2] = data.quaternion.z
        self.quat[3] = data.quaternion.w
    def run_flag_callback(self,data):
        self.run_flag = 1
    
    def encoder_callback(self,data):

        self.encoder1 = data.encoder3
        self.encoder3 = data.encoder4
        self.encoder_step1 = data.encoder_step1
        self.encoder_step2 = data.encoder_step2
        self.DAC_value1 = data.DAC_value1
        self.DAC_value2 = data.DAC_value2

    def image_callback(self,data):
        self.image_data = data.data 
        #rospy.loginfo('%x %x %x %x' %(self.image_data[0],self.image_data[1],self.image_data[10000],self.image_data[20000]))
    def thermal_image_callback(self,data):
        self.thermal_data = data.data
        
    def state_estimate_callback(self,data):
        self.psi[0] = data.psi_vicon[0]
        self.psi[1] = data.psi_vicon[1]
        self.psi[2] = data.psi_vicon[2]

        self.pos[0] = data.pos_vicon[0]
        self.pos[1] = data.pos_vicon[1]
        self.pos[2] = data.pos_vicon[2]

        self.vel[0] = data.vel[0]
        self.vel[1] = data.vel[1]
        self.vel[2] = data.vel[2]

        self.omega[0] = data.omega[0]
        self.omega[1] = data.omega[1]
        self.omega[2] = data.omega[2]

    def run_test(self):
        """"
        main loop 
        """

        File_name = time.strftime('%Y_%m_%d_%H_%M_%S')
    
        file = open('/home/bikebot/Documents/Data_Agribot/Motion_MPC_' + File_name + '.txt', 'w')
        #file_img = open('/home/bikebot/Documents/Data_Agribot/Img_' + File_name + '.txt', 'w')
        while not self.run_flag:
            pass

        rospy.loginfo('start controlling...')

        rate = rospy.Rate(100)
        t0 = rospy.Time.now().to_sec()

        while not rospy.is_shutdown():
            _Last = rospy.Time.now().to_sec()
            rot = Rotlib.from_quat(self.quat)
            rot_mat = rot.as_matrix()
            Rot = torch.tensor(rot_mat,dtype=torch.double)
            self.psi_adj = self.psi[0]/180.0*pi

            self.r_imu = self.pos[:2].clone()
            self.r_imu_dot = self.vel[:2].clone()
            self.v_back = self.encoder1/53.7377
            self.v_front = self.encoder3/54.1387
            self.angle_left = self.encoder_step1/2000.0*2*pi
            self.angle_right = self.encoder_step2/2000.0*2*pi
                
            if  (self.v_back<0.1 and self.v_back>-0.1) or (self.v_front<0.1 and self.v_front>-0.1):
                self.r_dot = Rot.mv(torch.tensor([self.v_back,0,0],dtype=torch.double)).float()[:2]
            else:
                self.r_dot = (self.vel[:2] + torch.cross(self.omega,self.p_c)[:2]).float()
                
            self.v_GPS = torch.norm(self.r_dot,p=2,dim=0)    
            self.r = self.pos[:2]+(Rot.mv(self.p_c)).float()[:2] - self.pos0[:2].float()
            self.r_abs_new = math.sqrt(self.r[0]**2+self.r[1]**2)


            #print(self.psi_adj)
            if self.t<200:
                print(self.t)
                if self.v_back<self.v_back_init and self.init_flag == 0:
                    rospy.loginfo('initializing')
                    self.psi_0 = self.psi_cur
                    self.pos0 = self.pos + (Rot.mv(self.p_c)).float()
                    
                    self.r = torch.zeros_like(self.r)
                    self.acc_index += 1
                    self.t = 0
                    self.t_init = 0
                    self.Rot0 = Rot
                    command_v_left =1.0*(1-math.exp(-0.008*self.acc_index))
                    phi_left = 0.0

                elif (self.v_back>self.v_back_init or self.init_flag == 1) and self.t_init<5:
                    self.init_flag = 1
                    rospy.loginfo('initializing2')
                    self.psi_0 = self.psi_cur
                    self.pos0 = self.pos + (Rot.mv(self.p_c)).float()
                    
                    self.r = torch.zeros_like(self.r)
                    self.acc_index += 1
                    self.t = 0
                    self.Rot0 = Rot
                    command_v_left = self.v_back_init
                    phi_left = 0.0
                    self.rd_arr_adj  = self.Rot0[:2,:2].mm(torch.tensor(self.rd_arr)).numpy()
                    self.cx = self.rd_arr_adj[0,:]
                    self.cy = self.rd_arr_adj[1,:]
                    # 计算一阶导数
                    for i in range(len(self.cx)-1):
                        self.dx[i] = (self.cy[i+1] - self.cy[i])/(self.cx[i+1] - self.cx[i])
                        self.dx[len(self.cx)-1] = self.dx[len(self.cx)-2]
                
                    # 计算二阶导数
                    for i in range(len(self.cx)-2):
                        self.ddx[i] = (self.cy[i+2] - 2*self.cy[i+1] + self.cy[i]) / (0.5 * (self.cx[i+2] - self.cx[i]))**2
                        self.ddx[len(self.cx)-2] = self.ddx[len(self.cx)-3]
                        self.ddx[len(self.cx)-1] = self.ddx[len(self.cx)-2]
                
                    # 计算偏航角
                    for i in range(len(self.cx)):
                        self.cyaw[i] = atan(self.dx[i])
                
                    # 计算曲率
                    for i in range(len(self.cx)):
                        self.ck[i] = self.ddx[i] / (1 + self.dx[i]**2)**1.5


                elif self.t_init>5:
                    #self.command_v_back = 0.8
                    self.init_flag = 2
                    self.idx = self.mpc.findIdx(self.r[0],self.r[1],self.rd_arr_adj[0,:],self.rd_arr_adj[1,:])
                    if (self.idx == len(self.rd_arr_adj[0,:])-1):
                        break
                    
                    tar_v = 1.0
                    tar_angle = atan(self.L_vert * self.ck[self.idx])

                    #MPC operation
                    (v, angle) = self.mpc.mpcControl(self.r[0].numpy(), self.r[1].numpy(), self.psi_adj.numpy(), self.v_back, self.angle_left,
                                    self.cx[self.idx], self.cy[self.idx], self.cyaw[self.idx], tar_v, tar_angle)
                    command_v_left = v
                    phi_left = angle/pi*180.0

                if abs(phi_left) > 0.1:
                    d = self.L_vert/math.tan(phi_left)
                    right_d  = d + self.dist_hori
                    phi_right = math.atan(self.L_vert/right_d)
                    command_v_right = command_v_left/d*right_d

                else:
                    phi_right = phi_left
                    command_v_right = command_v_left

                msg_vel = ros_hub_vel()
                msg_vel.hub_vel1 = float(command_v_left)
                msg_vel.hub_vel2 = float(command_v_right)
                #rospy.loginfo('send_vel: %f %f' %(msg_vel.hub_vel1,msg_vel.hub_vel2))
                self.pub_vel.publish(msg_vel)

                msg_turn = ros_step_pav()
                msg_turn.step_pos1 = float(phi_left)
                msg_turn.step_pos2 = float(phi_right)
                #rospy.loginfo('send_hub: %f %f' %(msg_turn.step_pos1,msg_turn.step_pos2))
                self.pub_turn.publish(msg_turn)


                _Now = rospy.Time.now().to_sec()
                file.write('{}  {} {} {} {} {} {} {} {} {} {} {} {} {} {} {} {} {} \n'.format(float(self.t),float(self.dt),float(self.psi_adj),float(self.v_back),float(self.v_front),float(self.v_GPS),\
                                                self.angle_left,self.angle_right,phi_left,phi_right,command_v_left,command_v_right,\
                                                    float(self.r[0]),float(self.r[1]),self.cx[self.idx],self.cy[self.idx],self.cyaw[self.idx],self.ck[self.idx]))


                while (_Now-_Last < self.dt_std):
                    _Now = rospy.Time.now().to_sec()
                self.dt = _Now-_Last
                self.mpc.dt = self.dt
                self.t += self.dt
                self.t_init += self.dt
            else:
                if self.over_index==0:
                    print('control program over...')
                self.over_index = 1
                msg_vel = ros_hub_vel()
                msg_vel.hub_vel1 = float(0)
                msg_vel.hub_vel2 = float(0)
                #rospy.loginfo('send_vel: %f %f' %(msg_vel.hub_vel1,msg_vel.hub_vel2))
                self.pub_vel.publish(msg_vel)

                msg_turn = ros_step_pav()
                msg_turn.step_pos1 = float(0)
                msg_turn.step_pos2 = float(0)
                #rospy.loginfo('send_hub: %f %f' %(msg_turn.step_pos1,msg_turn.step_pos2))
                self.pub_turn.publish(msg_turn)
                
        file.close()
        rospy.spin()


if __name__ == '__main__':
        rospy.init_node('controller')
        
        rospy.loginfo('welcome to control system...')
        controller = CONTROLLER()
        controller.run_test()
