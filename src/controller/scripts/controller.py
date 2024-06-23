#!/usr/bin/env python3

import torch
import time
import math
import rospy
import numpy as np
from sensor_msgs.msg import Imu
from std_msgs.msg import String
from geometry_msgs.msg import TransformStamped
from geometry_msgs.msg import Quaternion
from state_estimate.msg import state_IMU
from time_synchronization.msg import sync_gather
from serial_node.msg import ros_hub_vel,ros_step_pav
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
    p_c = -torch.tensor([390e-3,-340e-3,848.375e-3],dtype=torch.double)
    #L_vert = 1 #m
    

    psi = torch.zeros([3])
    pos = torch.zeros([3])
    vel = torch.zeros([3])

    omega = torch.zeros([3],dtype=torch.double)

    quat = torch.zeros([4])

    #state variables
    r_imu = torch.zeros([2])
    r_imu_dot = torch.zeros([2])
    r = torch.zeros([2])
    r_dot = torch.zeros([2])
    psi_cur = torch.zeros([1])
    
    rd_ddot = torch.zeros([2])
    rd_dot = torch.zeros([2])
    rd = torch.zeros([2])
    encoder1 = 0
    encoder3 = 0

    v_back = 0
    v_front = 0

    Rot = torch.zeros([3,3],dtype=torch.double)
    dt = 0.2
    t = 0
    run_flag = 0
    
    #command 
    command_v = torch.zeros([2])
    command_phi = torch.zeros([2])

    kp = 0.8
    kd = 0.2

     #observe
    g_origin = torch.zeros([2,2])
    g_inverse = torch.zeros([2,2])
    er = torch.zeros([2])
    ur = torch.zeros([2])
    u = torch.zeros([2])
    send_vel = torch.zeros([2])
    send_phi = torch.zeros([2])

    #accleration counter
    acc_cnt = 30
    acc_index = 0

    R_cmd = 1.2
    omega_cmd = 0.4

    pub_vel = rospy.Publisher('/cmd_hub_vel',ros_hub_vel,queue_size=10)
    pub_turn = rospy.Publisher('/cmd_step_pos',ros_step_pav,queue_size=10)
    pub_store = rospy.Publisher('/run_store',controll_message,queue_size=10)

    
    def __init__(self):
        rospy.Subscriber('/run_motion',String,self.run_flag_callback)
        rospy.Subscriber('/state_estimate/state_estimate',state_IMU,self.state_estimate_callback)
        rospy.Subscriber('/sync/gather',sync_gather,self.encoder_callback)
        rospy.Subscriber('/sync/vicon',TransformStamped,self.vicon_quaternion_callback)

        

    
    def run(self,rd_ddot,rd_dot,rd):
        """"
        main loop 
        """
        #todo
        #x= from estimate
        #y= from estimate
        #x_dot= fself.dt = _Now-_Last from ros
        File_name = time.strftime('%Y_%m_%d_%H_%M_%S')
        file = open('/home/bikebot/Documents/Data_Agribot/Exp_' + File_name + '.txt', 'w')
        time.sleep (1.5)
        while not self.run_flag:
            pass

        while not rospy.is_shutdown():
            _Last = rospy.Time.now().to_sec()
            self.r = self.pos[:2].clone()
            self.r_dot = self.vel[:2].clone()
            self.psi_cur = self.psi[0].clone()
   
            rot = Rotlib.from_quat(self.quat)
            a = rot.as_euler('zyx',degrees=True)
            rot_tmp = np.array(a)
            self.psi_cur = rot_tmp[0]
            rospy.loginfo('psi_cur:%f' %self.psi_cur)
            self.v_back = -self.encoder3 /2000*(3.1415926*45.3e-3)*508/390*200
            self.v_front = -self.encoder1 /2000*(3.1415926*45.3e-3)*508/390*100
            command_v,command_phi = self.command_v_and_phi(rd_ddot,rd_dot,rd, self.r_dot, self.r,self.kp,self.kd ,  self.psi_cur , self.v_back,self.v_front, self.dt)


            msg_vel = ros_hub_vel()
            msg_vel.hub_vel1 = float(command_v[0])
            msg_vel.hub_vel2 = float(command_v[1])
            rospy.loginfo('send_vel: %f %f' %(msg_vel.hub_vel1,msg_vel.hub_vel2))
            self.pub_vel.publish(msg_vel)

            msg_turn = ros_step_pav()
            msg_turn.step_pos1 = float(command_phi[0])
            msg_turn.step_pos2 = float(command_phi[1])
            rospy.loginfo('send_hub: %f %f' %(msg_turn.step_pos1,msg_turn.step_pos2))
            self.pub_turn.publish(msg_turn)
            
            _Now = rospy.Time.now().to_sec()
            while (_Now-_Last < self.dt):
                _Now = rospy.Time.now().to_sec()
            
        rospy.spin()

    def run_test(self):
        """"
        main loop 
        """
        #todo
        #x= from estimate
        #y= from estimate
        #x_dot= from estimate
        #y_dot= from estimate
        #psi_cur = from estimate 
        #v_back = 
        #dt = from ros

        File_name = time.strftime('%Y_%m_%d_%H_%M_%S')
        file = open('/home/bikebot/Documents/Data_Agribot/Exp_' + File_name + '.txt', 'w')
    
        while not self.run_flag:
            pass
        

        self.r_imu = self.pos[:2].clone()
        self.r_imu_dot = self.vel[:2].clone()
        rot = Rotlib.from_quat(self.quat)
        a = rot.as_euler('zyx',degrees=True)
        rot_tmp = np.array(a)
        self.psi_cur = torch.tensor([rot_tmp[0]])/180*pi

        Rot = torch.tensor([[self.psi_cur.cos(),-self.psi_cur.sin(),0],
                            [self.psi_cur.sin(),self.psi_cur.cos(),0],
                            [0,0,1]],dtype=torch.double)
        self.Rot = Rot
        self.r = self.r_imu + (Rot.mv(self.p_c)).float()[:2]
        self.rd = self.r

        rospy.loginfo('start controlling...')
        while not rospy.is_shutdown():
            _Last = rospy.Time.now().to_sec()

            if self.acc_index < self.acc_cnt:
                self.rd_ddot[0] = 0.6/self.acc_cnt
                self.rd_dot[0] = 0.6/self.acc_cnt * self.acc_index
                self.acc_index += 1
            else:
                self.rd_ddot[0] = 0
                self.rd_dot[0] = 0.6

            self.rd[0] += self.rd_dot[0]*self.dt

            self.rd_ddot[0] = 0

            self.rd[1] += 0
            self.rd_dot[1] = 0
            self.rd_ddot[1] = 0

            
            
            
            
            self.r_imu = self.pos[:2].clone()
            self.r_imu_dot = self.vel[:2].clone()
            rot = Rotlib.from_quat(self.quat)
            a = rot.as_euler('zyx',degrees=True)
            rot_tmp = np.array(a)
            self.psi_cur = torch.tensor([rot_tmp[0]])/180*pi

            Rot = torch.tensor([[self.psi_cur.cos(),-self.psi_cur.sin(),0],
                                [self.psi_cur.sin(),self.psi_cur.cos(),0],
                                [0,0,1]],dtype=torch.double)

            self.v_back = -self.encoder3 /2000*(3.1415926*45.3e-3)*508/390*200
            self.v_front = -self.encoder1 /2000*(3.1415926*45.3e-3)*508/390*100

            self.r = self.r_imu + (Rot.mv(self.p_c)).float()[:2]
            self.r_dot = Rot.mv(torch.tensor([self.v_back,0,0],dtype=torch.double)).float()[:2]

            rospy.loginfo('r: %f %f' %(self.r[0],self.r[1]))
            rospy.loginfo('r_dot: %f %f' %(self.r_dot[0],self.r_dot[1]))
            
            #self.psi_cur = self.psi[0].clone()/180*pi
            #self.psi_cur = torch.tensor([0.0])
            
            rospy.loginfo('psi:%f'%self.psi_cur)
            
            self.command_v,self.command_phi = self.command_v_and_phi(self.rd_ddot,self.rd_dot,self.rd, self.r_dot, self.r,self.kp,self.kd ,  self.psi_cur , self.v_back,self.v_front, self.dt)
            
            rospy.loginfo('command: %f,%f; %f,%f' %(self.command_v[0],self.command_v[1],self.command_phi[0],self.command_phi[1]))
            msg_vel = ros_hub_vel()
            msg_vel.hub_vel1 = float(self.command_v[0])
            msg_vel.hub_vel2 = float(self.command_v[1])
            rospy.loginfo('send_vel: %f %f' %(msg_vel.hub_vel1,msg_vel.hub_vel2))
            self.pub_vel.publish(msg_vel)

            msg_turn = ros_step_pav()
            msg_turn.step_pos1 = float(-self.command_phi[0])
            msg_turn.step_pos2 = float(-self.command_phi[1])
            rospy.loginfo('send_hub: %f %f' %(msg_turn.step_pos1,msg_turn.step_pos2))
            self.pub_turn.publish(msg_turn)

            msg_ctrl = controll_message()
            msg_ctrl.header.stamp = rospy.Time.now()
            msg_ctrl.psi = self.psi_cur

            msg_ctrl.g_origin = [float(self.g_origin[0,0]),float(self.g_origin[0,1]),float(self.g_origin[1,0]),float(self.g_origin[1,1])]

            msg_ctrl.g_inverse = [float(self.g_inverse[0,0]),float(self.g_inverse[0,1]),float(self.g_inverse[1,0]),float(self.g_inverse[1,1])]

            msg_ctrl.er = [float(self.er[0]),float(self.er[1])]
            msg_ctrl.ur = [float(self.ur[0]),float(self.ur[1])]
            msg_ctrl.u = [float(self.u[0]),float(self.u[1])]
            msg_ctrl.send_vel = [float(self.command_v[0]),float(self.command_v[1])]
            msg_ctrl.send_hub = [float(self.command_phi[0]),float(self.command_phi[1])]
            msg_ctrl.v_back = self.v_back
            self.pub_store.publish(msg_ctrl)
            _Now = rospy.Time.now().to_sec()
            print(float(msg_ctrl.psi))
            file.write('{}  {} {} {} {} {} {} {} {} {} {} {} {} {} {} {} {} {} {} {} {}\n '.format(_Now, float(msg_ctrl.psi),float(msg_ctrl.v_back),float(msg_ctrl.g_origin[0]),float(msg_ctrl.g_origin[1]),float(msg_ctrl.g_origin[2]),float(msg_ctrl.g_origin[3]),\
                float(msg_ctrl.g_inverse[0]),float(msg_ctrl.g_inverse[1]),float(msg_ctrl.g_inverse[2]),float(msg_ctrl.g_inverse[3]),float(msg_ctrl.er[0]),float(msg_ctrl.er[1]), \
                    float(msg_ctrl.ur[0]),float(msg_ctrl.ur[1]),float(msg_ctrl.u[0]),float(msg_ctrl.u[1]),float(msg_ctrl.send_vel[0]),float(msg_ctrl.send_vel[1]),float(msg_ctrl.send_hub[0]),float(msg_ctrl.send_hub[1])))
            
#             Header header

# float32 psi
# float32[] g_origin
# float32[] g_inverse
# float32[] er
# float32[] ur
# float32[] u
# float32[] send_vel
# float32[] send_hub        
            
            
            while (_Now-_Last < self.dt):
                _Now = rospy.Time.now().to_sec()
            self.dt = _Now-_Last
        file.close()
        rospy.spin()
        
    def run_circle(self):
        """"
        main loop 
        """
        #todo
        #x= from estimate
        #y= from estimate
        #x_dot= from estimate
        #y_dot= from estimate
        #psi_cur = from estimate 
        #v_back = 
        #dt = from ros

        File_name = time.strftime('%Y_%m_%d_%H_%M_%S')
        file = open('/home/bikebot/Documents/Data_Agribot/Exp_' + File_name + '.txt', 'w')
    
        while not self.run_flag:
            pass
        

        self.r_imu = self.pos[:2].clone()
        self.r_imu_dot = self.vel[:2].clone()
        rot = Rotlib.from_quat(self.quat)
        a = rot.as_euler('zyx',degrees=True)
        rot_tmp = np.array(a)
        self.psi_cur = torch.tensor([rot_tmp[0]])/180*pi

        Rot = torch.tensor([[self.psi_cur.cos(),-self.psi_cur.sin(),0],
                            [self.psi_cur.sin(),self.psi_cur.cos(),0],
                            [0,0,1]],dtype=torch.double)
        self.Rot = Rot
        self.r = self.r_imu + (Rot.mv(self.p_c)).float()[:2]
        self.rd = self.r

        rospy.loginfo('start controlling...')
        while not rospy.is_shutdown():
            _Last = rospy.Time.now().to_sec()

            if self.acc_index < self.acc_cnt:
                self.rd_ddot[0] = 0.3/self.acc_cnt
                self.rd_dot[0] = 0.3/self.acc_cnt * self.acc_index
                self.rd[0] += self.rd_dot[0]*self.dt
                rd_tmpx = self.r[0]
                rd_tmpy = self.r[1]
                self.acc_index += 1
            else:
                self.rd_ddot[0] = -self.R_cmd * self.omega_cmd**2 *math.sin(self.omega_cmd * self.dt)
                self.rd_ddot[1] = -self.R_cmd * self.omega_cmd**2 *math.cos(self.omega_cmd * self.dt)
                self.rd_dot[0] = self.R_cmd * self.omega_cmd *math.cos(self.omega_cmd * self.dt)
                self.rd_dot[1] = -self.R_cmd * self.omega_cmd *math.sin(self.omega_cmd * self.dt)
                self.rd[0] = self.R_cmd*math.sin(self.omega_cmd * self.t) + rd_tmpx
                self.rd[1] = self.R_cmd*math.cos(self.omega_cmd * self.t)-self.R_cmd + rd_tmpy

                print('t:',self.t)
                self.t += self.dt
            self.r_imu = self.pos[:2].clone()
            self.r_imu_dot = self.vel[:2].clone()
            rot = Rotlib.from_quat(self.quat)
            a = rot.as_euler('zyx',degrees=True)
            rot_tmp = np.array(a)
            self.psi_cur = torch.tensor([rot_tmp[0]])/180*pi

            Rot = torch.tensor([[self.psi_cur.cos(),-self.psi_cur.sin(),0],
                                [self.psi_cur.sin(),self.psi_cur.cos(),0],
                                [0,0,1]],dtype=torch.double)

            self.v_back = -self.encoder3 /2000*(3.1415926*45.3e-3)*508/390*200
            self.v_front = -self.encoder1 /2000*(3.1415926*45.3e-3)*508/390*100

            self.r = self.r_imu + (Rot.mv(self.p_c)).float()[:2]
            self.r_dot = Rot.mv(torch.tensor([self.v_back,0,0],dtype=torch.double)).float()[:2]

            rospy.loginfo('r: %f %f' %(self.r[0],self.r[1]))
            rospy.loginfo('r_dot: %f %f' %(self.r_dot[0],self.r_dot[1]))
            
            #self.psi_cur = self.psi[0].clone()/180*pi
            #self.psi_cur = torch.tensor([0.0])
            
            rospy.loginfo('psi:%f'%self.psi_cur)
            
            self.command_v,self.command_phi = self.command_v_and_phi(self.rd_ddot,self.rd_dot,self.rd, self.r_dot, self.r,self.kp,self.kd ,  self.psi_cur , self.v_back,self.v_front, self.dt)
            
            rospy.loginfo('command: %f,%f; %f,%f' %(self.command_v[0],self.command_v[1],self.command_phi[0],self.command_phi[1]))
            msg_vel = ros_hub_vel()
            msg_vel.hub_vel1 = float(self.command_v[0])
            msg_vel.hub_vel2 = float(self.command_v[1])
            rospy.loginfo('send_vel: %f %f' %(msg_vel.hub_vel1,msg_vel.hub_vel2))
            self.pub_vel.publish(msg_vel)

            msg_turn = ros_step_pav()
            msg_turn.step_pos1 = float(-self.command_phi[0])
            msg_turn.step_pos2 = float(-self.command_phi[1])
            rospy.loginfo('send_hub: %f %f' %(msg_turn.step_pos1,msg_turn.step_pos2))
            self.pub_turn.publish(msg_turn)

            msg_ctrl = controll_message()
            msg_ctrl.header.stamp = rospy.Time.now()
            msg_ctrl.psi = self.psi_cur

            msg_ctrl.g_origin = [float(self.g_origin[0,0]),float(self.g_origin[0,1]),float(self.g_origin[1,0]),float(self.g_origin[1,1])]

            msg_ctrl.g_inverse = [float(self.g_inverse[0,0]),float(self.g_inverse[0,1]),float(self.g_inverse[1,0]),float(self.g_inverse[1,1])]

            msg_ctrl.er = [float(self.er[0]),float(self.er[1])]
            msg_ctrl.ur = [float(self.ur[0]),float(self.ur[1])]
            msg_ctrl.u = [float(self.u[0]),float(self.u[1])]
            msg_ctrl.send_vel = [float(self.command_v[0]),float(self.command_v[1])]
            msg_ctrl.send_hub = [float(self.command_phi[0]),float(self.command_phi[1])]
            msg_ctrl.v_back = self.v_back
            self.pub_store.publish(msg_ctrl)
            _Now = rospy.Time.now().to_sec()
            print(float(msg_ctrl.psi))
            file.write('{}  {} {} {} {} {} {} {} {} {} {} {} {} {} {} {} {} {} {} {} {} {} {}\n '.format(_Now, float(msg_ctrl.psi),float(msg_ctrl.v_back),float(msg_ctrl.g_origin[0]),float(msg_ctrl.g_origin[1]),float(msg_ctrl.g_origin[2]),float(msg_ctrl.g_origin[3]),\
                float(msg_ctrl.g_inverse[0]),float(msg_ctrl.g_inverse[1]),float(msg_ctrl.g_inverse[2]),float(msg_ctrl.g_inverse[3]),float(msg_ctrl.er[0]),float(msg_ctrl.er[1]), \
                    float(msg_ctrl.ur[0]),float(msg_ctrl.ur[1]),float(msg_ctrl.u[0]),float(msg_ctrl.u[1]),float(msg_ctrl.send_vel[0]),float(msg_ctrl.send_vel[1]),float(msg_ctrl.send_hub[0]),float(msg_ctrl.send_hub[1])),float(self.r[0]),float(self.r[1]))
            
#             Header header

# float32 psi
# float32[] g_origin
# float32[] g_inverse
# float32[] er
# float32[] ur
# float32[] u
# float32[] send_vel
# float32[] send_hub        
            
            
            while (_Now-_Last < self.dt):
                _Now = rospy.Time.now().to_sec()
            self.dt = _Now-_Last
        file.close()
        rospy.spin()

    #callback functions
    def vicon_quaternion_callback(self,data):
        self.quat[0] = data.transform.rotation.x
        self.quat[1] = data.transform.rotation.y
        self.quat[2] = data.transform.rotation.z
        self.quat[3] = data.transform.rotation.w


    def run_flag_callback(self,data):
        self.run_flag = 1
    
    def encoder_callback(self,data):
        self.encoder1 = data.encoder1
        self.encoder3 = data.encoder3

    def state_estimate_callback(self,data):
        self.psi[0] = data.psi[0]
        self.psi[1] = data.psi[1]
        self.psi[2] = data.psi[2]

        self.pos[0] = data.pos[0]
        self.pos[1] = data.pos[1]
        self.pos[2] = data.pos[2]

        self.vel[0] = data.vel[0]
        self.vel[1] = data.vel[1]
        self.vel[2] = data.vel[2]

        self.omega[0] = data.omega[0]
        self.omega[1] = data.omega[1]
        self.omega[2] = data.omega[2]
    #calculating functions
        
    def command_v_and_phi(self,rd_ddot,rd_dot,rd, r_dot, r,kp,kd ,  psi_cur , v_back,v_front, dt):
        v_cur = torch.sqrt(r_dot[0]**2 + r_dot[1]**2)
        u = self.calc_u(rd_ddot,rd_dot,rd, r_dot, r,kp,kd ,  psi_cur , v_cur)
        rospy.loginfo('u: %f %f' %(u[0],u[1]))
        v_dot = u[0]
        psi_dot = u[1]
        command_v_back = v_cur+v_dot*dt
        # if command_v_back < 0.3:
        #       command_v_back = 0.3
        
        command_phi_left, command_phi_right = self.psidot2phi(psi_dot,v_back,v_front,self.L_vert)
        
        command_phi = torch.tensor([command_phi_left,command_phi_right])
        command_v_left = torch.sqrt(command_v_back**2 + (psi_dot*self.L_vert)**2)
        
        if command_v_left>2:
            command_v_left = 2
        #command_v_right = command_v_left*command_phi_left.sin()/command_phi_right.sin()
        command_v_right = torch.sqrt((command_v_back + psi_dot* self.dist_hori)**2 +(psi_dot*self.L_vert)**2 )
        
        command_v = torch.tensor([command_v_left,command_v_right])
        
        return command_v,command_phi
    
    
    def calc_u(self,rd_ddot,rd_dot,rd, r_dot, r,kp,kd ,  psi_cur , v_cur):
        """"
        calculate u vector: v_dot , psi_dot
        input: g 2*2
               ur 2*1
        output: u 2*1
        """
        g = self.g_mat(psi=psi_cur,v=v_cur)
        self.g_origin = g
        g_inv = self.inv_mat(g)
        self.g_inverse = g_inv
        u = g_inv.mv(self.calc_ur(rd_ddot,rd_dot,rd, r_dot, r,kp,kd))
        rospy.loginfo('u_origin %f %f'%(u[0],u[1]))
        if u[1].abs()>0.05:
            u[1] = 0.05*u[1].sign()
        if self.acc_index < self.acc_cnt:
            u[1] = 0
        self.u = u
        return u
        
    def calc_ur(self,rd_ddot,rd_dot,rd, r_dot,r,kp,kd ):
        """"
        use PID(PD) to update u_r
        input : demand: rd_ddot,rd_dot,rd, 
                measure: r_dot, r
                all are 2*1 torch tensor 
        kp,kd: hyper parameters
        """
        e_r = r-rd
        
        e_r_dot = r_dot - rd_dot
        ur = rd_ddot - kp*e_r  - kd*e_r_dot
        ur[0] *= 0.4
        self.er = e_r
        self.ur = ur
        #rospy.loginfo('r_dot, rd_dot %f,%f' %(r[1],rd[1]))
        rospy.loginfo('er: (%f,%f),er_dot:(%f %f),rd_ddot:(%f %f) ur: (%f,%f)'  %(e_r[0],e_r[1],e_r_dot[0],e_r_dot[1],rd_ddot[0],rd_ddot[1],ur[0],ur[1]))
        return ur
    
    def psidot2phi(self, psidot , v_back, v_front , L_vert):
        """"
        input: v_back,v_front: all take left side
               L_vert: vehicle intrinsic parameter
               psidot: from controller(u_psi)
        """
        #using back wheel to give phi command


        if v_back<0.2:
            v_back = 0.2
        tan_phi_1 = psidot*L_vert/v_back
        rospy.loginfo('tanphi1 %f' %tan_phi_1)
        phi_1 = tan_phi_1.atan()
        rospy.loginfo('phi1 %f'%phi_1)
        #using front wheel to give phi command
        # if v_front<0.2:
        #     v_front = 0.2
        # sin_phi_2 = psidot*L_vert/v_front
        # phi_2 = sin_phi_2.asin()
        # rospy.loginfo('phi2 %f' %phi_2)
        
        #average weight
        phi_left = phi_1
        #phi_right = torch.atan(L_vert*phi_left.tan()/(self.dist_hori*phi_left.tan()+L_vert))
        phi_right = phi_left
        phi_left = phi_left/pi*180
        phi_right = phi_right/pi*180
        rospy.loginfo('%f %f %f'%(psidot,v_back,tan_phi_1))
        return phi_left, phi_right
    
    def demand_retrieve(self,rd,dt):
        """"
        input : rd 3*2 torch tensor in plane problem
        """
        rd_dot1 = (rd[1,:]-rd[0,:])/dt
        rd_dot2 = (rd[2,:]-rd[1,:])/dt
        rd_ddot = (rd_dot2-rd_dot1)/dt
        return rd_ddot, rd_dot1


    def g_mat(self,psi,v):
        """"
        psi: torch element
        v: torch element
        output: g matrix 2*2
        """
        if v < 0.2:
            v = 0.2
        g = torch.zeros([2,2])
        g[0,0] = psi.cos()
        g[0,1] = -v*psi.sin()
        g[1,0] = psi.sin()
        g[1,1] = v*psi.cos()
        rospy.loginfo('g_origin: %f %f; %f %f' %(g[0,0],g[0,1],g[1,0],g[1,1]))
        return g
    
    def inv_mat(self , X):
        """"
        X has to be N*N
        output: inverse of X
        wrap function
        """
        #pseudo inverse matrix
        tmp  = torch.linalg.pinv(X)
        rospy.loginfo('g_inv %f %f %f %f'%(tmp[0,0],tmp[0,1],tmp[1,0],tmp[1,1]))
        return tmp
     
    
        
if __name__ == '__main__':
        rospy.init_node('controller')
        rospy.loginfo('welcome to control system...')
        controller = CONTROLLER()
        controller.run_circle()
