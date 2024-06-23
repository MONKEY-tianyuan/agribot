from PyQt5 import QtCore
from PyQt5.QtCore import QObject
import serial
import sys
import struct
from struct import unpack
import threading
import math
import time
from datetime import datetime
import gol
from ctypes import *

def bytearray2str(array):
    out_s = ''
    for i  in range(len(array)):
        out_s = out_s + ' '+(hex(int(array[i]))).upper()[2:].zfill(2)

    return out_s


class ReceiveThread(QtCore.QThread):
    def __init__(self, 
                 Ser,
                 dispContent) -> None:
        super(ReceiveThread,self).__init__()
        self.Ser = Ser
        self.dispContent = dispContent
        fname  = gol.get_value('fname')
        print('fname',fname)
        self.rfname = 'data/r'+fname
        self.init_time = gol.get_value('init_time')
        self.receive_buffer = 0
    def run(self):
        print('start receiving thread')
        f = open(self.rfname,'w')

        while True:
            tic = datetime.now()
            count = self.Ser.in_waiting
            if count >= 40:
                recv = self.Ser.read(count)
                #print('I receive',(recv),'len',len(recv))
                if recv[0] == 0xaa and recv[1] == 0xbb and recv[38] == 0xcc and recv[39] == 0xdd:
                    recv = recv[2:]
                    pos_int1 = byte2short(get_hex(recv[:2]))
                    spd_int1= byte2short(get_hex(recv[2:4]))
                    cur_int1 = byte2short(get_hex(recv[4:6]))

                    pos_int2 = byte2short(get_hex(recv[8:10]))
                    spd_int2= byte2short(get_hex(recv[10:12]))
                    cur_int2 = byte2short(get_hex(recv[12:14]))

                    steer_init_pos1_int = byte2int(get_hex(recv[16:20]))
                    steer_cur1_int = byte2short(get_hex(recv[20:22]))
                    steer_vel1_int = byte2short(get_hex(recv[22:24]))
                    steer_pos1_int = byte2short(get_hex(recv[24:26]))

                    steer_init_pos2_int = byte2int(get_hex(recv[26:30]))
                    steer_cur2_int = byte2short(get_hex(recv[30:32]))
                    steer_vel2_int = byte2short(get_hex(recv[32:34]))
                    steer_pos2_int = byte2short(get_hex(recv[34:36]))



                    motor_pos1 = pos_int1*0.1
                    motor_spd1 = spd_int1*10.0
                    motor_cur1 = cur_int1*0.01

                    motor_pos2 = pos_int2*0.1
                    motor_spd2 = spd_int2*10.0
                    motor_cur2 = cur_int2*0.01

                    steer_init_pos1 = steer_init_pos1_int * 0.01
                    steer_cur1 = steer_cur1_int * 0.01
                    steer_vel1 = steer_vel1_int/36.0
                    steer_pos1 = float(steer_pos1_int)

                    steer_init_pos2 = steer_init_pos2_int * 0.01
                    steer_cur2 = steer_cur2_int * 0.01
                    steer_vel2 = steer_vel2_int/36.0
                    steer_pos2 = float(steer_pos2_int)


                    # print('current',motor_cur1)
                    print('steer_init_pos 1',steer_init_pos1)
                    print('steer pos 1', steer_pos1)
                    motor_temp1 = recv[6]
                    motor_error1 = recv[7]

                    motor_temp2 = recv[14]
                    motor_error2 = recv[15]
                    t_now = datetime.now() 
                    f.write(str((t_now-self.init_time).total_seconds())+' '+\
                            str(motor_pos1) + ' ' + str(motor_spd1) + ' ' +str(motor_cur1) + ' ' + str(motor_temp1) +' ' + str(motor_error1)+ ' '+\
                            str(motor_pos2) + ' ' + str(motor_spd2) + ' ' +str(motor_cur2) + ' ' + str(motor_temp2) +' ' + str(motor_error2)+ ' '+\
                            str(steer_init_pos1) + ' ' +  str(steer_pos1) + ' ' + str(steer_vel1) + ' ' + str(steer_cur1) + ' ' +\
                            str(steer_init_pos2) + ' ' +  str(steer_pos2) + ' ' + str(steer_vel2) + ' ' + str(steer_cur2)    )
                    f.write('\n')
                self.Ser.flushInput()
            toc = datetime.now()
            sleep_time = 0.01-(toc-tic).total_seconds()
            #print('sleep time',sleep_time)
            if sleep_time>0:
                time.sleep(sleep_time)

            if not self.Ser.is_open:
                self.quit()
                f.close()
                return
        
class TransmitThread(QtCore.QThread):
    def __init__(self, 
                 Ser,
                 get_txbuffer,
                 ) -> None:
        super(TransmitThread,self).__init__()
        self.Ser = Ser

        fname  = gol.get_value('fname')
        self.sfname = 'data/s'+fname

        self.txbuffer = bytearray([0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFC])
        self.get_txbuffer = get_txbuffer
        self.t_loc = 0
        self.init_time = gol.get_value('init_time')
        self.init_cnt = 0
        self.brake_set_cnt = 0
        self.brake_release_cnt = 0
        self.rel_pos_cnt = self.brake_set_cnt = 0
        self.brake_release_cnt = 0
        self.rel_pos_cnt = 0

        self.steer_motion_flag = 0
        self.prev_steer_motion_flag = 0

        self.rel_pos_vel = 0
        self.rel_pos_pos = 0

        self.prev_rel_pos_vel = 0
        self.prev_rel_pos_pos = 0
        self.zero = 0
        
        self.vel_steer = 0
        self.pos_steer = 0

        self.t_count = 0

    def run(self):
        print('start transmitting thread')
        f = open(self.sfname,'w')
        while True:
            tic = datetime.now()
            send_buffer = None
            steer_send_buffer = None
            print_str = str(0)
            motion = gol.get_value('motion')
            if motion is None:
                motion = bytearray([0x00])
            steer_motion = gol.get_value('steer_motion')
            motion_flag = gol.get_value('motion_flag')
            steer_motion_flag = gol.get_value('steer_motion_flag')
            self.steer_motion_flag = steer_motion_flag
            vel = 0
            match motion_flag:
                case 1:
                    amplitude = float(gol.get_value('amp'))
                    T = float(gol.get_value('T'))
                    omega = 2*math.pi/T
                    vel = amplitude*math.sin(omega*self.t_loc) #+ 3000.0
                    send_buffer = motion + struct.pack('f',vel)
                    print_str = str(vel)
                case 2:
                    vel = float(gol.get_value('amp'))
                    send_buffer = motion + struct.pack('f',vel)
                    print_str = str(vel)
                case 3:
                    self.t_loc = 0
                    current = float(gol.get_value('cur'))
                    send_buffer = motion + struct.pack('f',current)
                    print_str = str(current)
                case 4:
                    amplitude = float(gol.get_value('amp'))
                    T = float(gol.get_value('T'))
                    omega = 2*math.pi/T
                    vel = amplitude*math.sin(omega*self.t_loc)
                    send_buffer = motion + struct.pack('f',vel)
                    print_str = str(vel)
                case 5:
                    vel = float(gol.get_value('amp'))
                    send_buffer = motion + struct.pack('f',vel)
                    print_str = str(vel)

            

            match steer_motion_flag:
                case 1:
                    # send rear_send_buffer to the global
                    vel_steer = 0
                    pos_steer = 0
                    gol.set_value('send_buffer',send_buffer)

                    #set time to zero
                    self.t_loc = 0

                case 2:
                    # send rear_send_buffer to the global
                    vel_steer = 0
                    pos_steer = 0
                    gol.set_value('send_buffer',send_buffer)
                    
                    #set time to zero
                    self.t_loc = 0
                    
                    
                case 3:
                    vel_steer = int(gol.get_value('steer_max_speed'))
                    amplitude = float(gol.get_value('steer_amp'))
                    omega = float(gol.get_value('steer_freq'))*2*math.pi
                    pos_steer = int(amplitude*math.sin(omega*self.t_loc))
                    # sign = (math.sin(omega*self.t_loc))
                    # if sign > 0:
                    #     pos_steer = int(amplitude)
                    # else:
                    #     pos_steer = int(-amplitude)
                    print('hub vel:',vel)
                    print('send_buffer',send_buffer)
                    steer_send_buffer = steer_motion + struct.pack('H',self.vel_steer) + struct.pack('i',self.pos_steer)
                    self.send_buffer(send_buffer,steer_send_buffer)

                case 4:
                    vel_steer = int(gol.get_value('steer_max_speed'))
                    pos_steer = int(gol.get_value('steer_amp'))
                    steer_send_buffer = steer_motion + struct.pack('H',vel_steer) + struct.pack('i',pos_steer)
                    self.send_buffer(send_buffer,steer_send_buffer)
                    
                case 5:
                    # send rear_send_buffer to the global
                    gol.set_value('send_buffer',send_buffer)
                    vel_steer = 0
                    pos_steer = 0

                    #set time to zero
                    self.t_loc = 0
                case 6:
                    # send rear_send_buffer to the global
                    gol.set_value('send_buffer',send_buffer)
                    vel_steer = 0
                    pos_steer = 0

                    #set time to zero
                    self.t_loc = 0


            if self.t_count % 3 == 0:
                self.vel_steer = vel_steer
                self.pos_steer = pos_steer

            str2 = str((datetime.now()-self.init_time).total_seconds())
            
            str_save = str2 + ' ' + str(int(motion[0])) +' ' + print_str +\
                        str(self.vel_steer) +' ' + str(self.pos_steer)
            f.write(str_save)
            f.write('\n')

            
            self.t_loc += 0.02
            self.t_count += 1
            toc = datetime.now()
            sleep_time = 0.02-(toc-tic).total_seconds()

            if sleep_time>0:
                time.sleep(sleep_time)

            if not self.Ser.is_open:
                self.quit()
                f.close()
                return
    
    def send_buffer(self,rear_send_buffer,steer_send_buffer):
        Send_buffer = rear_send_buffer + rear_send_buffer + steer_send_buffer + steer_send_buffer
        print('send',(Send_buffer))
        self.Ser.write(Send_buffer)


def get_hex(bytes):
    # print(bytes)
    l = ''
    for byte in bytes:
        tmp = hex(int(byte))
        # print('tmp',tmp)
        l_tmp = tmp[2:]
        l += l_tmp
    #print(l)
    return l

def byte2short(l):
    # print('l',l)
    return c_short(int(l,16)).value

def byte2int(l):
    return c_int(int(l,16)).value