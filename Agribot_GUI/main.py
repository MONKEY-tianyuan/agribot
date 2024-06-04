import os
import shlex
import signal
import subprocess
import sys
import time
import typing
import math

import time
import platform
import cv2
import numpy as np
import roslaunch
import rospy
import yaml
from cv_bridge import CvBridge, CvBridgeError
from loguru import logger
from PyQt5 import QtGui
from PyQt5.QtCore import (QEvent, QObject, QSize, Qt, QThread, QTimer,
                          pyqtSignal, pyqtSlot, qDebug)
from PyQt5.QtGui import QFont, QIcon, QImage, QMouseEvent, QPixmap, QVector3D
from PyQt5.QtWidgets import (QApplication, QFileDialog, QHBoxLayout, QLabel,
                             QMainWindow, QPushButton, QVBoxLayout, QWidget)
from roslaunch import parent, rlutil
from sensor_msgs.msg import Image
from state_estimate.msg import state_IMU
from std_msgs.msg import String
from serial_node.msg import ros_hub_vel,ros_step_pav,stm2ros_tao

class RosNode(QThread):

    RGB_image_signal = pyqtSignal()
    thermal_image_signal = pyqtSignal()
    RGB_image_opencv = None
    thermal_image_opencv = None
    RGB_showImage = None
    thermal_showImage = None
    image_index = 0
    thermal_index = 0

    def __init__(self):
        super(RosNode,self).__init__()
        rospy.init_node('qt_ros_node')
        self.loop_rate = rospy.Rate(10,reset=True)
        self.stop_flag = False
        self.uuid = rlutil.get_or_generate_uuid(None,False)
        roslaunch.configure_logging(self.uuid)
        self.init_launch = parent.ROSLaunchParent(self.uuid,['/home/bikebot/IEKF_ws/src/state_estimate/launch/estimate.launch','/home/bikebot/serial_node_ws/src/serial_node/launch/agribot.launch','/home/bikebot/GPS_ws/src/xsens_ros_mti_driver/launch/xsens_mti_node.launch'])
        self.sync_launch = parent.ROSLaunchParent(self.uuid,['/home/bikebot/sync2_ws/src/time_synchronization/launch/sync.launch'])
        self.RGB_launch = parent.ROSLaunchParent(self.uuid,['/home/bikebot/camera_fromtx2_ws/src/pointgrey_camera_driver-noetic-devel/pointgrey_camera_driver/launch/camera.launch'])
        self.thermal_launch = parent.ROSLaunchParent(self.uuid,['/home/bikebot/flir_camera_ws/src/flir_camera_driver-noetic-devel/spinnaker_camera_driver/launch/thermal_camera.launch'])
        self.IEKF_calibrate_publish = rospy.Publisher('/init_proc',String,queue_size=1)
        self.motion_start_publish = rospy.Publisher('/run_motion',String,queue_size=1)
        self.curtain_up_publish = rospy.Publisher('/curtain_up',String,queue_size=1)
        self.curtain_down_publish = rospy.Publisher('/curtain_down',String,queue_size=1)
        self.curtain_stay_publish = rospy.Publisher('/curtain_stay',String,queue_size=1)
        #self.vel_publish = rospy.Publisher('/cmd_hub_vel',String,queue_size=1)
        self.pub_vel = rospy.Publisher('/cmd_hub_vel',ros_hub_vel,queue_size=10)
        self.pub_turn = rospy.Publisher('/cmd_step_pos',ros_step_pav,queue_size=10)

        

        rospy.Subscriber('/flir_camera_1/image_color',Image,self.callback_RGB_image)
        rospy.Subscriber('/thermal_camera/image_raw',Image,self.callback_thermal_image)
        

    

    
    
    def publish_IEKF_calibrate(self):
        message = String()
        message.data = ' '
        self.IEKF_calibrate_publish.publish(message)

    def publish_motion_run(self):
        message = String()
        message.data = ' '
        self.motion_start_publish.publish(message)

    def callback_RGB_image(self,data):
        self.image_index += 1

        if self.image_index == 10:
            bridge = CvBridge()
            self.RGB_image_opencv = bridge.imgmsg_to_cv2(data,'bgr8')
            self.RGB_image_opencv = cv2.resize(self.RGB_image_opencv,(640,480))
            self.RGB_image_opencv = cv2.cvtColor(self.RGB_image_opencv,cv2.COLOR_BGR2RGB)
            self.RGB_showImage = QtGui.QImage(self.RGB_image_opencv.data,self.RGB_image_opencv.shape[1],self.RGB_image_opencv.shape[0],QtGui.QImage.Format_RGB888)
            self.RGB_image_signal.emit()
            self.image_index = 0

    def callback_thermal_image(self,data):
        self.thermal_index += 1
        if self.thermal_index == 20:
            bridge = CvBridge()
            self.thermal_image_opencv = bridge.imgmsg_to_cv2(data,'mono8')
            self.thermal_image_opencv = cv2.resize(self.thermal_image_opencv,(640,480))
            #self.thermal_image_opencv = cv2.cvtColor(self.thermal_image_opencv,cv2.CAP_PVAPI_PIXELFORMAT_MONO8)
            self.thermal_showImage = QtGui.QImage(self.thermal_image_opencv.data,self.thermal_image_opencv.shape[1],self.thermal_image_opencv.shape[0],QtGui.QImage.Format_Grayscale8)
            self.RGB_image_signal.emit()
            self.thermal_index = 0
        
    def run(self) -> None:
        while not rospy.is_shutdown() and not self.stop_flag:
            self.loop_rate.sleep()
    




class MainWindow(QWidget):
    cmd_hub_vel1 = 0.0
    cmd_hub_vel2 = 0.0
    cmd_step_pos1 = 0.0
    cmd_step_pos2 = 0.0
    L_vert = 1.2319
    dist_hori = 0.8636 #unit: m

    def __init__(self):
        super(MainWindow, self).__init__()
        self.layout1 = QHBoxLayout(self)
        self.layout2 = QVBoxLayout(self) 
        self.layout3 = QHBoxLayout(self)
        #timer to show images
        self.timer_RGB_camera = QTimer()
        self.timer_thermal_camera = QTimer()
        
        self.curtain_status = 0
        self.curtain_loop_flag = 0
        #buttons
        self.setWindowTitle('Agribot Control Panel')
        self.vel_up_btn = QPushButton('vel_up')
        self.vel_down_btn = QPushButton('vel_down')
        self.left_btn = QPushButton('left')
        self.right_btn = QPushButton('right')
        self.sync_btn = QPushButton('sync')
        self.RGB_btn = QPushButton('RGB_camera')
        self.thermal_btn = QPushButton('thermal_camera')
        self.IEKF_btn = QPushButton('IMU_calibrate')
        self.control_btn = QPushButton('motion START')
        self.RGB_stop_btn = QPushButton('RGB_STOP')
        self.thermal_stop_btn = QPushButton('thermal_STOP')
        self.curtain_up = QPushButton('curtain_up')
        self.curtain_down = QPushButton('curtain_down')
        self.curtain_stay = QPushButton('curtain_stay')
        self.curtain_loop_btn = QPushButton('curtain_loop')
        self.curtain_loop_btn.setAutoRepeat(True)
        self.T = time.time()

        self.label_RGB = QLabel()
        self.label_RGB.setFixedSize(641,481)

        self.label_thermal = QLabel()
        self.label_thermal.setFixedSize(641,481)
        
        self.layout2.addWidget(self.IEKF_btn)
        self.layout2.addWidget(self.control_btn)
        self.layout2.addWidget(self.RGB_btn)
        self.layout2.addWidget(self.thermal_btn)

        self.layout2.addWidget(self.sync_btn)

        self.layout2.addWidget(self.vel_up_btn)
        self.layout2.addWidget(self.vel_down_btn)
        self.layout2.addWidget(self.left_btn)
        self.layout2.addWidget(self.right_btn)

        self.layout2.addWidget(self.RGB_stop_btn)
        self.layout2.addWidget(self.thermal_stop_btn)

        self.layout2.addWidget(self.curtain_up)
        self.layout2.addWidget(self.curtain_down)
        self.layout2.addWidget(self.curtain_stay)
        self.layout2.addWidget(self.curtain_loop_btn)

        self.layout1.addLayout(self.layout2)
        self.layout1.addWidget(self.label_RGB)

        #self.layout3.addLayout(self.layout1)
        self.layout1.addWidget(self.label_thermal)

        
        
        self.ros_node = RosNode()
        self.ros_node.start()

        #initinalize launch files
        self.ros_node.init_launch.start()
        

        self.RGB_btn.clicked.connect(self.RGB_roslaunch)
        self.thermal_btn.clicked.connect(self.thermal_roslaunch)
        self.sync_btn.clicked.connect(self.sync_roslaunch)
        self.IEKF_btn.clicked.connect(self.ros_node.publish_IEKF_calibrate)
        self.control_btn.clicked.connect(self.ros_node.publish_motion_run)
        self.RGB_stop_btn.clicked.connect(self.RGB_stop)
        self.thermal_stop_btn.clicked.connect(self.thermal_stop)
        self.vel_up_btn.clicked.connect(self.vel_up)
        self.vel_down_btn.clicked.connect(self.vel_down)
        self.left_btn.clicked.connect(self.left)
        self.right_btn.clicked.connect(self.right)
        self.timer_RGB_camera.timeout.connect(self.show_RGB_camera)
        self.timer_thermal_camera.timeout.connect(self.show_thermal_camera)
        self.curtain_up.clicked.connect(self.curtain_up_pub)
        self.curtain_down.clicked.connect(self.curtain_down_pub)
        self.curtain_stay.clicked.connect(self.curtain_stay_pub)
        
        self.curtain_loop_btn.clicked.connect(self.curtain_loop_pub)
    def curtain_loop_pub(self):
        if self.curtain_status == 0:
            message = String()
            message.data = ' '
            self.ros_node.curtain_up_publish.publish(message)
            self.curtain_status = 1 
        while 1:
            T = time.time()
            while time.time()-T < 5:
                pass
            T = time.time()
            if self.curtain_status == 1:
                message = String()
                message.data = ' '
                self.ros_node.curtain_stay_publish.publish(message)
                while time.time()-T <2:
                    pass
                message = String()
                message.data = ' '
                self.ros_node.curtain_down_publish.publish(message)
                self.curtain_status = 2
            elif self.curtain_status == 2:
                message = String()
                message.data = ' '
                self.ros_node.curtain_stay_publish.publish(message)
                while time.time()-T <2:
                    pass
                message = String()
                message.data = ' '
                self.ros_node.curtain_up_publish.publish(message)
                self.curtain_status = 1



    def curtain_up_pub(self):
        message = String()
        message.data = ' '
        self.ros_node.curtain_up_publish.publish(message)
    
    def curtain_down_pub(self):
        message = String()
        message.data = ' '
        self.ros_node.curtain_down_publish.publish(message)
    
    def curtain_stay_pub(self):
        message = String()
        message.data = ' '
        self.ros_node.curtain_stay_publish.publish(message)

    def vel_up(self):
        self.cmd_hub_vel1 += 0.1
        phi_left = self.cmd_step_pos1/180*math.pi
        phi_right = self.cmd_step_pos2/180*math.pi
        if phi_left > 0.75:
            phi_left = 0.75
        if phi_left<-0.75:
            phi_left = -0.75
        if phi_left > 0.05 or phi_left<-0.05:
            d = self.L_vert/math.tan(phi_left)
            right_d  = d + self.dist_hori
            phi_right = math.atan(self.L_vert/right_d)
            self.cmd_hub_vel2 = self.cmd_hub_vel1/d*right_d
        else:
            phi_right = phi_left
            self.cmd_hub_vel2 = self.cmd_hub_vel1

        self.cmd_step_pos1 = phi_left/math.pi*180
        self.cmd_step_pos2 = phi_right/math.pi*180
        msg_vel = ros_hub_vel()
        msg_vel.hub_vel1 = float(self.cmd_hub_vel1)
        msg_vel.hub_vel2 = float(self.cmd_hub_vel2)
        #rospy.loginfo('send_vel: %f %f' %(msg_vel.hub_vel1,msg_vel.hub_vel2))
        self.ros_node.pub_vel.publish(msg_vel)
        msg_turn = ros_step_pav()
        msg_turn.step_pos1 = float(self.cmd_step_pos1)
        msg_turn.step_pos2 = float(self.cmd_step_pos2)
        #rospy.loginfo('send_hub: %f %f' %(msg_turn.step_pos1,msg_turn.step_pos2))
        self.ros_node.pub_turn.publish(msg_turn)
    
    def vel_down(self):
        self.cmd_hub_vel1 -= 0.1
        phi_left = self.cmd_step_pos1/180*math.pi
        phi_right = self.cmd_step_pos2/180*math.pi
        if phi_left > 0.75:
            phi_left = 0.75
        if phi_left<-0.75:
            phi_left = -0.75
        if phi_left > 0.05 or phi_left<-0.05:
            d = self.L_vert/math.tan(phi_left)
            right_d  = d + self.dist_hori
            phi_right = math.atan(self.L_vert/right_d)
            self.cmd_hub_vel2 = self.cmd_hub_vel1/d*right_d
        else:
            phi_right = phi_left
            self.cmd_hub_vel2 = self.cmd_hub_vel1

        self.cmd_step_pos1 = phi_left/math.pi*180
        self.cmd_step_pos2 = phi_right/math.pi*180
        msg_vel = ros_hub_vel()
        msg_vel.hub_vel1 = float(self.cmd_hub_vel1)
        msg_vel.hub_vel2 = float(self.cmd_hub_vel2)
        #rospy.loginfo('send_vel: %f %f' %(msg_vel.hub_vel1,msg_vel.hub_vel2))
        self.ros_node.pub_vel.publish(msg_vel)
        msg_turn = ros_step_pav()
        msg_turn.step_pos1 = float(self.cmd_step_pos1)
        msg_turn.step_pos2 = float(self.cmd_step_pos2)
        #rospy.loginfo('send_hub: %f %f' %(msg_turn.step_pos1,msg_turn.step_pos2))
        self.ros_node.pub_turn.publish(msg_turn)
    
    def left(self):
        self.cmd_step_pos1 += 5
        phi_left = self.cmd_step_pos1/180*math.pi
        phi_right = self.cmd_step_pos2/180*math.pi
        if phi_left > 0.75:
            phi_left = 0.75
        if phi_left<-0.75:
            phi_left = -0.75
        if phi_left > 0.05 or phi_left<-0.05:
            d = self.L_vert/math.tan(phi_left)
            right_d  = d + self.dist_hori
            phi_right = math.atan(self.L_vert/right_d)
            self.cmd_hub_vel2 = self.cmd_hub_vel1/d*right_d
        else:
            phi_right = phi_left
            self.cmd_hub_vel2 = self.cmd_hub_vel1

        self.cmd_step_pos1 = phi_left/math.pi*180
        self.cmd_step_pos2 = phi_right/math.pi*180
        msg_vel = ros_hub_vel()
        msg_vel.hub_vel1 = float(self.cmd_hub_vel1)
        msg_vel.hub_vel2 = float(self.cmd_hub_vel2)
        #rospy.loginfo('send_vel: %f %f' %(msg_vel.hub_vel1,msg_vel.hub_vel2))
        self.ros_node.pub_vel.publish(msg_vel)
        msg_turn = ros_step_pav()
        msg_turn.step_pos1 = float(self.cmd_step_pos1)
        msg_turn.step_pos2 = float(self.cmd_step_pos2)
        #rospy.loginfo('send_hub: %f %f' %(msg_turn.step_pos1,msg_turn.step_pos2))
        self.ros_node.pub_turn.publish(msg_turn)
    
    def right(self):
        self.cmd_step_pos1 -= 5
        phi_left = self.cmd_step_pos1/180*math.pi
        phi_right = self.cmd_step_pos2/180*math.pi
        if phi_left > 0.75:
            phi_left = 0.75
        if phi_left<-0.75:
            phi_left = -0.75
        if phi_left > 0.05 or phi_left<-0.05:
            d = self.L_vert/math.tan(phi_left)
            right_d  = d + self.dist_hori
            phi_right = math.atan(self.L_vert/right_d)
            self.cmd_hub_vel2 = self.cmd_hub_vel1/d*right_d
        else:
            phi_right = phi_left
            self.cmd_hub_vel2 = self.cmd_hub_vel1

        self.cmd_step_pos1 = phi_left/math.pi*180
        self.cmd_step_pos2 = phi_right/math.pi*180
        msg_vel = ros_hub_vel()
        msg_vel.hub_vel1 = float(self.cmd_hub_vel1)
        msg_vel.hub_vel2 = float(self.cmd_hub_vel2)
        #rospy.loginfo('send_vel: %f %f' %(msg_vel.hub_vel1,msg_vel.hub_vel2))
        self.ros_node.pub_vel.publish(msg_vel)
        msg_turn = ros_step_pav()
        msg_turn.step_pos1 = float(self.cmd_step_pos1)
        msg_turn.step_pos2 = float(self.cmd_step_pos2)
        #rospy.loginfo('send_hub: %f %f' %(msg_turn.step_pos1,msg_turn.step_pos2))
        self.ros_node.pub_turn.publish(msg_turn)


    def RGB_roslaunch(self):
        self.ros_node.RGB_launch.start()
        self.timer_RGB_camera.start(50)

    def thermal_roslaunch(self):
        self.ros_node.thermal_launch.start()
        self.timer_thermal_camera.start(50)

    def sync_roslaunch(self):
        self.ros_node.sync_launch.start()

    def RGB_stop(self):
        self.ros_node.RGB_launch.shutdown()
    
    def thermal_stop(self):
        self.ros_node.thermal_launch.shutdown()

    def show_RGB_camera(self):

        self.label_RGB.setPixmap(QPixmap(self.ros_node.RGB_showImage))

    def show_thermal_camera(self):
        self.label_thermal.setPixmap(QPixmap(self.ros_node.thermal_showImage))

    def closeEvent(self, a0) -> None:
        self.ros_node.stop_flag = True
        self.ros_node.init_launch.shutdown()
        self.ros_node.RGB_launch.shutdown()
        self.ros_node.thermal_launch.shutdown()
        self.ros_node.sync_launch.shutdown()

        self.timer_RGB_camera.stop()
        self.timer_thermal_camera.stop()


if __name__ == '__main__':
    app = QApplication(sys.argv)
    w = MainWindow()
    
    w.show()
    sys.exit(app.exec_())
    while (w.curtain_loop_flag == False):
        pass
    w.curtain_loop()
