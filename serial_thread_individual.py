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
        
        self.vel_steer_1 = 0
        self.pos_steer_1 = 0

        self.vel_steer_2 = 0
        self.pos_steer_2 = 0

        self.t_count = 0

    def run(self):
        print('start transmitting thread')
        f = open(self.sfname,'w')
        while True:
            tic = datetime.now()
            send_buffer_1 = None
            send_buffer_2 = None
            steer_send_buffer_1 = None
            steer_send_buffer_2 = None
            print_str_1 = str(0)
            print_str_2 = str(0)
            
            motion_1 = gol.get_value('motion_1')
            steer_motion_1 = gol.get_value('steer_motion_1')
            motion_flag_1 = gol.get_value('motion_flag_1')
            steer_motion_flag_1 = gol.get_value('steer_motion_flag_1')
            self.steer_motion_flag_1 = steer_motion_flag_1

            motion_2 = gol.get_value('motion_2')
            steer_motion_2 = gol.get_value('steer_motion_2')
            motion_flag_2 = gol.get_value('motion_flag_2')
            steer_motion_flag_2 = gol.get_value('steer_motion_flag_2')
            self.steer_motion_flag_2 = steer_motion_flag_2

            # print('motion type',motion_1,motion_2,steer_motion_1,steer_motion_2)
            # if steer_motion_flag_1 == 1 or steer_motion_flag_2 == 1:
            #     print('motion flag type',motion_flag_1,motion_flag_2,steer_motion_flag_1,steer_motion_flag_2)


            if motion_1 is None:
                motion_1 = bytearray([0x00])

            if motion_2 is None:
                motion_2 = bytearray([0x00])

            if steer_motion_1 is None:
                steer_motion_1 = bytearray([0x00])

            if steer_motion_2 is None:
                steer_motion_2 = bytearray([0x00])



            # send_buffer_1 = gol.get_value('send_buffer_1')
            # send_buffer_2 = gol.get_value('send_buffer_2')
            # steer_send_buffer_1 = gol.get_value('steer_send_buffer_1')
            # steer_send_buffer_2 = gol.get_value('steer_send_buffer_2')

            # if send_buffer_1 is None:
            #     send_buffer_1 = bytearray([0x04]) + struct.pack('f',20.0)
            # if send_buffer_2 is None:
            #     send_buffer_2 = bytearray([0x04]) + struct.pack('f',20.0)
            # if steer_send_buffer_1 is None:
            steer_send_buffer_1 = bytearray([0x00]) + struct.pack('H',0) + struct.pack('i',0)
            # if steer_send_buffer_2 is None:
            steer_send_buffer_2 = bytearray([0x00]) + struct.pack('H',0) + struct.pack('i',0)

            vel1 = 0
            vel2 = 0
            match motion_flag_1:
                case 1:
                    amplitude = float(gol.get_value('amp_1'))
                    T = float(gol.get_value('T_1'))
                    omega = 2*math.pi/T
                    vel1 = amplitude*math.sin(omega*self.t_loc) #+ 3000.0
                    send_buffer_1 = motion_1 + struct.pack('f',vel1)
                    print_str_1 = str(vel1)
                    #break
                case 2:
                    vel1 = float(gol.get_value('amp_1'))
                    send_buffer_1 = motion_1 + struct.pack('f',vel1)
                    print('vel1',vel1)
                    print_str_1 = str(vel1)
                    #break
                case 3:
                    self.t_loc = 0
                    current = float(gol.get_value('cur_1'))
                    send_buffer_1 = motion_1 + struct.pack('f',current)
                    print_str_1 = str(current)
                    #break
                case 4:
                    amplitude = float(gol.get_value('amp_1'))
                    T = float(gol.get_value('T_1'))
                    omega = 2*math.pi/T
                    vel1 = amplitude*math.sin(omega*self.t_loc)
                    send_buffer_1 = motion_1 + struct.pack('f',vel1)
                    print_str_1 = str(vel1)
                    #break
                case 5:
                    vel1 = float(gol.get_value('amp_1'))
                    send_buffer_1 = motion_1 + struct.pack('f',vel1)
                    print_str_1 = str(vel1)
                    #break

            match motion_flag_2:
                case 1:
                    amplitude = float(gol.get_value('amp_2'))
                    T = float(gol.get_value('T_2'))
                    omega = 2*math.pi/T
                    vel2 = amplitude*math.sin(omega*self.t_loc) #+ 3000.0
                    send_buffer_2 = motion_2 + struct.pack('f',vel2)
                    print_str_2 = str(vel2)
                    #break
                case 2:
                    vel2 = float(gol.get_value('amp_2'))
                    send_buffer_2 = motion_2 + struct.pack('f',vel2)
                    print('vel2',vel2)
                    print_str_2 = str(vel2)
                   # break
                case 3:
                    self.t_loc = 0
                    current = float(gol.get_value('cur_2'))
                    send_buffer_2 = motion_2 + struct.pack('f',current)
                    print_str_2 = str(current)
                    #break
                case 4:
                    amplitude = float(gol.get_value('amp_2'))
                    T = float(gol.get_value('T_2'))
                    omega = 2*math.pi/T
                    vel2 = amplitude*math.sin(omega*self.t_loc)
                    send_buffer_2 = motion_2 + struct.pack('f',vel2)
                    print_str_2 = str(vel2)
                    #break
                case 5:
                    vel2 = float(gol.get_value('amp_2'))
                    send_buffer_2 = motion_2 + struct.pack('f',vel2)
                    print_str_2 = str(vel2)
                    #break
            

            match steer_motion_flag_1:
                case 0:
                    vel_steer_1 = 0
                    pos_steer_1 = 0
                    #break
                case 1:
                    # send rear_send_buffer to the global
                    vel_steer_1 = 0
                    pos_steer_1 = 0
                    gol.set_value('send_buffer_1',send_buffer_1)
                    gol.set_value('send_buffer_2',send_buffer_2)

                    #set time to zero
                    self.t_loc = 0
                    #break

                case 2:
                    # send rear_send_buffer to the global
                    vel_steer_1 = 0
                    pos_steer_1 = 0
                    gol.set_value('send_buffer_1',send_buffer_1)
                    gol.set_value('send_buffer_2',send_buffer_2)
                    
                    #set time to zero
                    self.t_loc = 0
                    #break
                    
                    
                case 3:
                    vel_steer_1 = int(gol.get_value('steer_max_speed_1'))
                    amplitude = float(gol.get_value('steer_amp_1'))
                    omega = float(gol.get_value('steer_freq_1'))*2*math.pi
                    pos_steer_1 = int(amplitude*math.sin(omega*self.t_loc))
                    # sign = (math.sin(omega*self.t_loc))
                    # if sign > 0:
                    #     pos_steer = int(amplitude)
                    # else:
                    #     pos_steer = int(-amplitude)
                    #print('hub vel:',vel)
                    #print('send_buffer',send_buffer)
                    steer_send_buffer_1 = steer_motion_1 + struct.pack('H',self.vel_steer_1) + struct.pack('i',self.pos_steer_1)
                    #break

                case 4:
                    vel_steer_1 = int(gol.get_value('steer_max_speed_1'))
                    pos_steer_1 = int(gol.get_value('steer_amp_1',500))
                    steer_send_buffer_1 = steer_motion_1 + struct.pack('H',vel_steer_1) + struct.pack('i',pos_steer_1)
                    # print('pos_steer_1',pos_steer_1)
                    #break

                    
                case 5:
                    # send rear_send_buffer to the global
                    gol.set_value('send_buffer_1',send_buffer_1)
                    gol.set_value('send_buffer_2',send_buffer_2)
                    vel_steer_1 = 0
                    pos_steer_1 = 0

                    #set time to zero
                    self.t_loc = 0
                    #break
                case 6:
                    # send rear_send_buffer to the global
                    gol.set_value('send_buffer_1',send_buffer_1)
                    gol.set_value('send_buffer_2',send_buffer_2)
                    vel_steer_1 = 0
                    pos_steer_1 = 0

                    #set time to zero
                    self.t_loc = 0
                    #break


            match steer_motion_flag_2:
                case 0:
                    vel_steer_2 = 0
                    pos_steer_2 = 0
                    #break
                case 1:
                    # send rear_send_buffer to the global
                    vel_steer_2 = 0
                    pos_steer_2 = 0
                    gol.set_value('send_buffer_1',send_buffer_1)
                    gol.set_value('send_buffer_2',send_buffer_2)

                    #set time to zero
                    self.t_loc = 0
                    #break

                case 2:
                    # send rear_send_buffer to the global
                    vel_steer_2 = 0
                    pos_steer_2 = 0
                    gol.set_value('send_buffer_1',send_buffer_1)
                    gol.set_value('send_buffer_2',send_buffer_2)
                    
                    #set time to zero
                    self.t_loc = 0
                    
                    #break
                    
                case 3:
                    vel_steer_2 = int(gol.get_value('steer_max_speed_2'))
                    amplitude = float(gol.get_value('steer_amp_2'))
                    omega = float(gol.get_value('steer_freq_2'))*2*math.pi
                    pos_steer_2 = int(amplitude*math.sin(omega*self.t_loc))
                    # sign = (math.sin(omega*self.t_loc))
                    # if sign > 0:
                    #     pos_steer = int(amplitude)
                    # else:
                    #     pos_steer = int(-amplitude)
                    #print('hub vel:',vel)
                    #print('send_buffer',send_buffer)
                    steer_send_buffer_2 = steer_motion_2 + struct.pack('H',self.vel_steer_2) + struct.pack('i',self.pos_steer_2)
                    self.send_buffer_individual(send_buffer_1,send_buffer_2,steer_send_buffer_1,steer_send_buffer_2)
                    #break
                case 4:
                    vel_steer_2 = int(gol.get_value('steer_max_speed_2',500))
                    pos_steer_2 = int(gol.get_value('steer_amp_2'))
                    steer_send_buffer_2 = steer_motion_2 + struct.pack('H',vel_steer_2) + struct.pack('i',pos_steer_2)
                    self.send_buffer_individual(send_buffer_1,send_buffer_2,steer_send_buffer_1,steer_send_buffer_2)
                    # print('pos_steer_2',pos_steer_2)
                    #break
                    
                case 5:
                    # send rear_send_buffer to the global
                    gol.set_value('send_buffer_1',send_buffer_1)
                    gol.set_value('send_buffer_2',send_buffer_2)
                    vel_steer_2 = 0
                    pos_steer_2 = 0

                    #set time to zero
                    self.t_loc = 0
                    #break
                case 6:
                    # send rear_send_buffer to the global
                    gol.set_value('send_buffer_1',send_buffer_1)
                    gol.set_value('send_buffer_2',send_buffer_2)
                    vel_steer_2 = 0
                    pos_steer_2 = 0

                    #set time to zero
                    self.t_loc = 0
                    #break

            if self.t_count % 3 == 0:
                self.vel_steer_1 = vel_steer_1
                self.pos_steer_1 = pos_steer_1
                self.vel_steer_2 = vel_steer_2
                self.pos_steer_2 = pos_steer_2

            str2 = str((datetime.now()-self.init_time).total_seconds())
            
            str_save = str2 + ' ' + str(int(motion_1[0])) +' ' + print_str_1 +\
                        str(self.vel_steer_1) +' ' + str(self.pos_steer_1) +\
                        ' ' + str(int(motion_2[0])) +' ' + print_str_2 +\
                        str(self.vel_steer_2) +' ' + str(self.pos_steer_2)
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
    
    def send_buffer_individual(self,rear_send_buffer1,rear_send_buffer2,steer_send_buffer1,steer_send_buffer2):
        Send_buffer = rear_send_buffer1 + rear_send_buffer2 + steer_send_buffer1 + steer_send_buffer2
        # print('send',(Send_buffer))
        self.Ser.write(Send_buffer)

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