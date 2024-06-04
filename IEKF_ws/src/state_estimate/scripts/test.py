#!/usr/bin/env python
import rospy
from random import random
from state_estimate.msg import state_IMU
from scipy.spatial.transform import Rotation as Rotlib
import torch


quat = torch.tensor([0,0,0,1],dtype=torch.float32)
print(torch.tensor((quat.tolist()+quat.tolist())[3:]))
tmp = Rotlib.from_quat(quat)
matrix = tmp.as_matrix()
matrix_torch = torch.tensor(matrix)
print(matrix_torch)