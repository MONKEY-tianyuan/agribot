#!/usr/bin/env python3

import torch
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


class CONTROLLER:
    """"
    input: demand positions: at least 3 demand points to calculate 2nd derivative of rd
    output: comand velocity; comand turn angle
    feedback measurement: x,y, 1st-derivative of x,y  , 2nd-derivative of x,y  ,
                          orientation w.r.t World Frame ,  
    """
    #car intrisic parameter
    dist_hori = 0.8636 #unit: m
    L_vert = 1.2319
    L_diag = math.sqrt(dist_hori**2 + L_vert**2)
    p_c = torch.tensor([-0.5588,0.2286,-0.91948],dtype=torch.double)
    #L_vert = 1 #m

    k = 0.15  # 前视距离系数
    Lfc = 0.9  # 前视距离
    image_data = torch.zeros([9437184])

    psi = torch.zeros([3])
    pos = torch.zeros([3])
    vel = torch.zeros([3])

    t_init = 0

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

    rd_ddot = torch.zeros([2])
    rd_dot = torch.zeros([2])
    rd = torch.zeros([2])
    rd_arr = None
    rd_arr_adj = None
    encoder1 = 0
    encoder3 = 0

    v_back = 0
    v_back_init = 0.6
    v_front = 0
    encoder_step1 = 0
    encoder_step2 = 0

    command_v_back = 0
    Rot = torch.zeros([3,3],dtype=torch.double)
    dt = 0.01
    dt_std = 0.01
    t = 0
    run_flag = 0
    init_flag = 0
    #command 
    command_v = torch.zeros([2])
    command_phi = torch.zeros([2])

    

     #observe
    g_origin = torch.zeros([2,2])
    g_inverse = torch.zeros([2,2])
    er = torch.zeros([2])
    ur = torch.zeros([2])
    u = torch.zeros([2])
    send_vel = torch.zeros([2])
    send_phi = torch.zeros([2])

    #accleration counter
    acc_cnt = 400
    acc_index = 0
    index = 0
    R_cmd = 1.2
    omega_cmd = 0.2
    over_index = 0

    target_ind = 0.0
    di = 0.0
    interval_left = 0
    interval_right = 0

    pub_vel = rospy.Publisher('/cmd_hub_vel',ros_hub_vel,queue_size=10)
    pub_turn = rospy.Publisher('/cmd_step_pos',ros_step_pav,queue_size=10)
    pub_store = rospy.Publisher('/run_store',controll_message,queue_size=10)
    pub_img = rospy.Publisher('/camera_ctrl/image_color',Image,queue_size=10)
    
    def __init__(self):
        rospy.Subscriber('/run_motion',String,self.run_flag_callback)
        rospy.Subscriber('/state_estimate/state_estimate',state_IMU,self.state_estimate_callback)
        rospy.Subscriber('/encoder',stm2ros_tao,self.encoder_callback)
        rospy.Subscriber('/sync/vicon',TransformStamped,self.vicon_quaternion_callback)
        rospy.Subscriber('/filter/quaternion',QuaternionStamped,self.xsens_callback)
        rospy.Subscriber('/camera/image_color',Image,self.image_callback)

        straight_forward_x = torch.arange(0,15,0.1).reshape([150,1])
        print(straight_forward_x.size())
        straight_forward_y = torch.zeros_like(straight_forward_x)
        straight_forward = torch.cat([straight_forward_x,straight_forward_y],1)
        print(straight_forward)


        straight_backward_x = torch.arange(15,0,-0.1).reshape([150,1])
        straight_backward_y = straight_forward_y
        straight_backward = torch.cat([straight_backward_x,straight_backward_y],1)
        #print(straight_backward)

        i = 0
        R = 2.2
        w = 0.1/R
        curve_ccw_x = torch.zeros([int(math.pi/w)+2,1])
        curve_ccw_y = torch.zeros([int(math.pi/w)+2,1])

        for i in range(0,int(math.pi/w)+2):
            curve_ccw_x[i] = R*math.sin(w*i)+15
            curve_ccw_y[i] = R - R*math.cos(w*i)

        curve_ccw = torch.cat([curve_ccw_x,curve_ccw_y],1)
        print(curve_ccw_x)
        print(curve_ccw_y)
        print(curve_ccw.size())

        curve_cw_x = torch.zeros([int(math.pi/w)+2,1])
        curve_cw_y = torch.zeros([int(math.pi/w)+2,1])

        for i in range(0,int(math.pi/w)+2):
            curve_cw_x[i] = -R*math.sin(w*i)
            curve_cw_y[i] = R - R*math.cos(w*i)

        curve_cw = torch.cat([curve_cw_x,curve_cw_y],1)
        print(curve_ccw_x)
        print(curve_ccw_y)
        print(curve_ccw.size())

        path_profile = torch.cat([straight_forward,curve_ccw,straight_backward+torch.tensor([0,2*R]),\
                                curve_cw+torch.tensor([0,2*R]),straight_forward+torch.tensor([0,4*R])],0)
        print(path_profile.size())

        self.rd_arr = path_profile.t()
        print(self.rd_arr.size())
        self.interval_right = self.rd_arr.size()[1]
    
    

    def run_test(self):
        """"
        main loop 
        """

        File_name = time.strftime('%Y_%m_%d_%H_%M_%S')
    
        file = open('/home/bikebot/Documents/Data_Agribot/Motion_' + File_name + '.txt', 'w')
        file_img = open('/home/bikebot/Documents/Data_Agribot/Img_' + File_name + '.txt', 'w')
        while not self.run_flag:
            pass

        rospy.loginfo('start controlling...')

        rate = rospy.Rate(200)
        t0 = rospy.Time.now().to_sec()
        while not rospy.is_shutdown():
            _Last = rospy.Time.now().to_sec()
            rot = Rotlib.from_quat(self.quat)
            rot_mat = rot.as_matrix()
            Rot = torch.tensor(rot_mat,dtype=torch.double)
            self.psi_adj = self.psi[0]/180.0*pi
            #print(self.psi_adj)
            self.v_back = self.encoder1/53.7377
            self.v_front = self.encoder3/54.1387
                
            if  (self.v_back<0.1 and self.v_back>-0.1) or (self.v_front<0.1 and self.v_front>-0.1):
                self.r_dot = Rot.mv(torch.tensor([self.v_back,0,0],dtype=torch.double)).float()[:2]
            else:
                self.r_dot = (self.vel[:2] + torch.cross(self.omega,self.p_c)[:2]).float()
                
                
            self.r = self.pos[:2]+(Rot.mv(self.p_c)).float()[:2] - self.pos0[:2].float()
            self.r_abs_new = math.sqrt(self.r[0]**2+self.r[1]**2)
            if self.t<200 and self.target_ind < self.interval_right:
                if self.v_back<self.v_back_init and self.init_flag == 0:
                    rospy.loginfo('initializing')
                    self.psi_0 = self.psi_cur
                    self.pos0 = self.pos + (Rot.mv(self.p_c)).float()
                    self.r = torch.zeros_like(self.r)
                    command_v_left = 0.7*(1-math.exp(-0.008*self.acc_index))
                    command_v_right = command_v_left
                    self.command_v[0] = command_v_left
                    self.command_v[1] = command_v_right
                    self.acc_index += 1
                    self.t = 0
                    self.t_init=0
                    self.Rot0 = Rot
                    self.rd_arr_adj  = self.Rot0[:2,:2].mm(self.rd_arr.double()).float()
                    command_phi_left = 0
                    command_phi_right = 0
                elif (self.v_back>self.v_back_init or self.init_flag == 1) and self.t_init<3.0:
                    self.init_flag = 1
                    rospy.loginfo('initializing2')
                    self.psi_0 = self.psi_cur
                    self.pos0 = self.pos + (Rot.mv(self.p_c)).float()
                    
                    self.r = torch.zeros_like(self.r)
                    self.acc_index += 1
                    self.t = 0
                    self.Rot0 = Rot
                    command_v_left = self.v_back_init
                    command_v_right = command_v_left
                    command_phi_left = 0
                    command_phi_right = 0
                    self.rd_arr_adj  = self.Rot0[:2,:2].mm(self.rd_arr.double()).float()
                    print(self.rd_arr_adj.size())
                elif self.t_init>=3.0:
                    self.init_flag = 1
                    command_v_left = 0.5
                    
                    tmp_delta = self.di
                    self.di, self.target_ind = self.pure_pursuit_control(self.rd_arr_adj[0,:],self.rd_arr_adj[1,:], self.target_ind)
                    if (self.di-tmp_delta>0.05):
                        self.di = tmp_delta+0.05
                    elif (self.di-tmp_delta<-0.05):
                        self.di = tmp_delta-0.05
                    self.index = self.index+1
                    command_phi_left,command_phi_right = self.di2phi(self.di)
                    if command_phi_left<-0.05/pi*180.0 or command_phi_left>0.05/pi*180.0:
                        d = self.L_vert/math.tan(command_phi_left/180.0*pi)
                        right_d  = d + self.dist_hori
                    #phi_right = math.atan(self.L_vert/right_d)
                        command_v_right = command_v_left/d*right_d
                    else:
                        command_v_right = command_v_left
                    self.command_v[0] = command_v_left
                    self.command_v[1] = command_v_right 

                # if (self.r_abs_new - self.r_abs_old >= 0.4):
                #     msg_image = Image()
                #     msg_image.data = self.image_data
                #     msg_image.header.stamp = rospy.Time.now()
                #     msg_image.header.seq += 1
                #     msg_image.height = 1536
                #     msg_image.width = 2048
                #     msg_image.encoding = 'bgr8'
                #     self.pub_img.publish(msg_image)
                #     self.r_abs_old = self.r_abs_new
                #     print('image taken once...')
                
                
                #rospy.loginfo('command: %f,%f; %f,%f' %(self.command_v[0],self.command_v[1],self.command_phi[0],self.command_phi[1]))
                msg_vel = ros_hub_vel()
                msg_vel.hub_vel1 = float(self.command_v[0])
                msg_vel.hub_vel2 = float(self.command_v[1])
                #rospy.loginfo('send_vel: %f %f' %(msg_vel.hub_vel1,msg_vel.hub_vel2))
                self.pub_vel.publish(msg_vel)

                msg_turn = ros_step_pav()
                msg_turn.step_pos1 = float(command_phi_left)
                msg_turn.step_pos2 = float(command_phi_right)
                #rospy.loginfo('send_hub: %f %f' %(msg_turn.step_pos1,msg_turn.step_pos2))
                self.pub_turn.publish(msg_turn)

                msg_ctrl = controll_message()
                msg_ctrl.header.stamp = rospy.Time.now()
                msg_ctrl.psi = self.psi_adj

                msg_ctrl.g_origin = [float(self.g_origin[0,0]),float(self.g_origin[0,1]),float(self.g_origin[1,0]),float(self.g_origin[1,1])]

                msg_ctrl.g_inverse = [float(self.g_inverse[0,0]),float(self.g_inverse[0,1]),float(self.g_inverse[1,0]),float(self.g_inverse[1,1])]

                msg_ctrl.er = [float(self.er[0]),float(self.er[1])]
                msg_ctrl.ur = [float(self.ur[0]),float(self.ur[1])]
                msg_ctrl.u = [float(self.u[0]),float(self.u[1])]
                msg_ctrl.send_vel = [float(self.command_v[0]),float(self.command_v[1])]
                msg_ctrl.send_hub = [float(self.command_phi[0]),float(self.command_phi[1])]
                msg_ctrl.v_back = self.v_back
                if self.index < 580:

                    self.rd = self.rd_arr_adj[:,self.index]
                _Now = rospy.Time.now().to_sec()
                #print(float(msg_ctrl.psi))
                file.write('{}  {} {} {} {} {} {} {} {} {} {} {} {} {} {} {} {} {} {} {} {} {} {} {} {} {} {} {} {} {} {} {} {} {} {}\n '.format(float(self.t), float(msg_ctrl.psi),float(msg_ctrl.v_back),float(self.di),float(self.target_ind),float(msg_ctrl.g_origin[2]),float(msg_ctrl.g_origin[3]),\
                    float(msg_ctrl.g_inverse[0]),float(msg_ctrl.g_inverse[1]),float(msg_ctrl.g_inverse[2]),float(msg_ctrl.g_inverse[3]),float(msg_ctrl.er[0]),float(msg_ctrl.er[1]), \
                        float(msg_ctrl.ur[0]),float(msg_ctrl.ur[1]),float(msg_ctrl.u[0]),float(msg_ctrl.u[1]),float(msg_ctrl.send_vel[0]),float(msg_ctrl.send_vel[1]),float(msg_ctrl.send_hub[0]),float(msg_ctrl.send_hub[1]),float(self.rd[0]),float(self.rd[1]),float(self.r[0]),float(self.r[1]),float(self.r_dot[0]),float(self.r_dot[1]),\
                            float(self.rd_ddot[0]),float(self.rd_ddot[1]),float(self.rd_dot[0]),float(self.rd_dot[1]),\
                                float(self.encoder1),float(self.encoder3),float(self.encoder_step1),float(self.encoder_step2)))
                #for index in range(9437184):
                    #file_img.write('{} '.format(self.image_data[index]))
                #file_img.write('\n')
                
                while (_Now-_Last < self.dt_std):
                    _Now = rospy.Time.now().to_sec()
                self.dt = _Now-_Last
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
        file_img.close()
        rospy.spin()
        
        
    

    #callback functions
    def vicon_quaternion_callback(self,data):
        #self.quat[0] = data.transform.rotation.x
        #self.quat[1] = data.transform.rotation.y
        #self.quat[2] = data.transform.rotation.z
        #self.quat[3] = data.transform.rotation.w

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


    def image_callback(self,data):
        self.image_data = data.data 
        #rospy.loginfo('%x %x %x %x' %(self.image_data[0],self.image_data[1],self.image_data[10000],self.image_data[20000]))

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
    #calculating functions
    def pure_pursuit_control(self, cx, cy, pind):

        ind = self.calc_target_index( cx, cy)

        if pind >= ind:
            ind = pind

        if ind < len(cx):
            tx = cx[ind]
            ty = cy[ind]
        else:
            tx = cx[-1]
            ty = cy[-1]
            ind = len(cx) - 1

        alpha = math.atan2(ty - self.r[1], tx - self.r[0]) - self.psi_adj

        if self.v_back < 0:  # back
            alpha = math.pi - alpha

        Lf = self.k * math.sqrt(self.r_dot[0]**2 + self.r_dot[1]**2) + self.Lfc
        delta = math.atan2(2.0 * self.L_vert * math.sin(alpha) / Lf, 1.0)
        print(delta)

        return delta, ind

    def calc_target_index(self,  cx, cy):
        # 搜索最临近的路点
        cx_tmp = cx[self.interval_left:self.interval_right]
        cy_tmp = cy[self.interval_left:self.interval_right]
        dx = [self.r[0] - icx for icx in cx_tmp]
        dy = [self.r[1] - icy for icy in cy_tmp]
        d = [abs(math.sqrt(idx ** 2 + idy ** 2)) for (idx, idy) in zip(dx, dy)]
        ind = d.index(min(d))+self.interval_left


        # if (100-(ind-self.interval_left)<5):
        #     self.interval_left = ind
        #     self.interval_right = self.interval_left + 100
        #print(self.interval_left,self.interval_right)
        print("before looking  ahead: ",ind)

        L = 0.0

        Lf = self.k * math.sqrt(self.r_dot[0]**2 + self.r_dot[1]**2) + self.Lfc

        while Lf > L and (ind + 1) < len(cx):
            dx = cx[ind + 1] - cx[ind]
            dy = cx[ind + 1] - cx[ind]
            L += math.sqrt(dx ** 2 + dy ** 2)
            ind += 1
        
        print('after looking ahead: ',ind)

        
        return ind
    def di2phi(self,di):
        phi_left = di
        if phi_left > 0.75:
            phi_left = 0.75
        if phi_left<-0.75:
            phi_left = -0.75
        if phi_left > 0.05 or phi_left<-0.05:
            d = self.L_vert/math.tan(phi_left)
            right_d  = d + self.dist_hori
            phi_right = math.atan(self.L_vert/right_d)
        else:
            phi_right = phi_left

        phi_left = phi_left/pi*180
        phi_right = phi_right/pi*180
        return phi_left, phi_right
    
        
if __name__ == '__main__':
        rospy.init_node('controller')
        
        rospy.loginfo('welcome to control system...')
        controller = CONTROLLER()
        controller.run_test()
