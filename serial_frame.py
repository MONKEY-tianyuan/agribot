from outputUI6 import *
from serial_thread_individual import ReceiveThread, TransmitThread
import serial
import struct
import math
import gol
from datetime import datetime

class SerialFrame(Ui_MainWindow):
    def __init__(self,
                 parent = None) -> None:
        super(SerialFrame,self).__init__(parent)
        

        self.ReceiveThread = None
        self.TransmitThread = None
        self.send_buffer_total = None
        self.send_buffer_steering = None

    def closeEvent(self):
        if self.ser.is_open:
            gol.set_value('motion_1',bytearray([0x04]))
            gol.set_value('motion_flag_1',3)

            gol.set_value('motion_2',bytearray([0x04]))
            gol.set_value('motion_flag_2',3)
            gol.set_value('cur',20.0)
            self.ser.close()
            print('serial status is:',self.ser.is_open)
            self.ReceiveThread.quit()
            self.TransmitThread.quit()
    
    def serialState(self):
        if not self.ser.is_open:
            try:
                self.ser.open()
                print('serial status is:',self.ser.is_open)
                gol.set_value('init_time',datetime.now())
                self.ReceiveThread = ReceiveThread(self.ser,None)
                self.TransmitThread = TransmitThread(self.ser,self.get_txbuffer)
                self.ReceiveThread.start()
                self.TransmitThread.start()

            except Exception:
                print(Exception)
                return
        else:
            self.ser.close()
    
    

    def get_txbuffer(self):
        match self.motion_flag:
            case 1:
                amplitude = float(self.lineEdit.text())
                omega = 2*math.pi/float(self.lineEdit_2.text())
                vel = amplitude*math.sin(omega*self.t)
                self.send_buffer = self.motion + struct.pack('f',vel)
                
            case 2:
                vel = float(self.lineEdit_3.text())
                self.send_buffer = self.motion + struct.pack('f',vel)
            case 3:
                current = float(self.lineEdit_4.text())
                self.send_buffer = self.motion + struct.pack('f',current)
            case 4:
                amplitude = float(self.lineEdit_5.text())
                omega = 2*math.pi/float(self.lineEdit_6.text())
                pos = amplitude*math.sin(omega*self.t)
                self.send_buffer = self.motion + struct.pack('f',pos)
            case 5:
                pos = float(self.lineEdit_7.text())
                self.send_buffer = self.motion + struct.pack('f',pos)

        self.send_buffer += self.send_buffer

        match self.steer_motion_flag:
            case 1:
                vel = 0
                pos = 0
                self.send_buffer_total = self.steer_motion + struct.pack('f',vel) + struct.pack('f',pos)
            
            case 2:
                vel = float(self.lineEdit_14.text())
                pos = float(self.lineEdit_12.text())
                self.send_buffer_total = self.steer_motion + struct.pack('f',vel) + struct.pack('f',pos)
            
            case 3:
                vel = float(self.lineEdit_13.text())
                amplitude = float(self.lineEdit_10.text())
                omega = 2*math.pi/float(self.lineEdit_8.text())
                pos = amplitude*math.sin(omega*self.t)
                self.send_buffer_total = self.steer_motion + struct.pack('f',vel) + struct.pack('f',pos)
            
            case 4:
                vel = float(self.lineEdit_13.text())
                pos = float(self.lineEdit_9.text())
                self.send_buffer_total = self.steer_motion + struct.pack('f',vel) + struct.pack('f',pos)
            
            case 5:
                vel = 0
                pos = 0
                self.send_buffer_total = self.steer_motion + struct.pack('f',vel) + struct.pack('f',pos)
            
            case 6:
                vel = 0
                pos = 0
                self.send_buffer_total = self.steer_motion + struct.pack('f',vel) + struct.pack('f',pos)
            
        self.send_buffer_steering += self.send_buffer_steering

        self.send_buffer_total = self.send_buffer + self.send_buffer_steering
        print('send_buffer_total',self.send_buffer_total)