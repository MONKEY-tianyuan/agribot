import torch
import math
import matplotlib.pyplot as plt
import os

os.environ["KMP_DUPLICATE_LIB_OK"] = "TRUE"

a = torch.tensor([[1.0,2,3,4],[4,3,2,1]])
b = torch.tensor([[5.0,6,7,8],[8,7,6,5]])
c= torch.cat([a,b],dim=1)
print(c)

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
R = 2.5
w = 0.1/R
d = 0.6
R2 = 6
w2 = 0.1/R2
curve_ccw_x = torch.zeros([int(math.pi/w)+2,1])
curve_ccw_y = torch.zeros([int(math.pi/w)+2,1])

for i in range(0,int(math.pi/w)+2):
    curve_ccw_x[i] = R*math.sin(w*i)+15
    curve_ccw_y[i] = 4 - R*math.cos(w*i)

curve_ccw = torch.cat([curve_ccw_x,curve_ccw_y],1)
print(curve_ccw_x)
print(curve_ccw_y)
print(curve_ccw.size())

curve_cw_x = torch.zeros([int(math.pi/w)+2,1])
curve_cw_y = torch.zeros([int(math.pi/w)+2,1])

for i in range(0,int(math.pi/w)+2):
    curve_cw_x[i] = -R*math.sin(w*i)
    curve_cw_y[i] = 4 - R*math.cos(w*i)

curve_cw = torch.cat([curve_cw_x,curve_cw_y],1)
print(curve_ccw_x)
print(curve_ccw_y)
print(curve_ccw.size())

path_profile = torch.cat([straight_forward,curve_ccw,straight_backward+torch.tensor([0,8.0]),\
                          curve_cw+torch.tensor([0,8.0]),straight_forward+torch.tensor([0,16.0])],0)
print(path_profile.size())

a = path_profile.t()
print(a.size()[1])

fig = plt.figure(num=1)
plt.plot(path_profile[:,0],path_profile[:,1])


curve_trans1_ccw_x = torch.zeros([int(math.acos((R2+d/2)/(R+R2))/w2)+2,1])
curve_trans1_ccw_y = torch.zeros([int(math.acos((R2+d/2)/(R+R2))/w2)+2,1])

for i in range(0,int(math.acos((R2+d/2)/(R+R2))/w2)+2):
    curve_trans1_ccw_x[i] = 15 + R2*math.cos(math.pi/2-w2*i)
    curve_trans1_ccw_y[i] = -R2 + R2*math.sin(math.pi/2-w2*i)
curve_trans1_ccw = torch.cat([curve_trans1_ccw_x,curve_trans1_ccw_y],1)

curve_turn_ccw_x = torch.zeros([int((2*math.acos((R2+d/2)/(R+R2))+math.pi)/w)+2,1])
curve_turn_ccw_y = torch.zeros([int((2*math.acos((R2+d/2)/(R+R2))+math.pi)/w)+2,1])

for i in range(0,int((2*math.acos((R2+d/2)/(R+R2))+math.pi)/w)+2):
    curve_turn_ccw_x[i] = 15 + math.sqrt((R2+R)**2 - (R2+d/2)**2) + R*math.cos(w*i-math.pi/2-math.acos((R2+d/2)/(R+R2)))
    curve_turn_ccw_y[i] = d/2 + R*math.sin(w*i-math.pi/2-math.acos((R2+d/2)/(R+R2)))
curve_turn_ccw = torch.cat([curve_turn_ccw_x,curve_turn_ccw_y],1)


curve_trans2_ccw_x = torch.zeros([int(math.acos((R2+d/2)/(R+R2))/w2)+2,1])
curve_trans2_ccw_y = torch.zeros([int(math.acos((R2+d/2)/(R+R2))/w2)+2,1])


for i in range(0,int(math.acos((R2+d/2)/(R+R2))/w2)+2):
    curve_trans2_ccw_x[i] = 15 + R2*math.cos(-w2*i-math.pi/2+math.acos((R2+d/2)/(R+R2)))
    curve_trans2_ccw_y[i] = d + R2 + R2*math.sin(-w2*i-math.pi/2+math.acos((R2+d/2)/(R+R2)))
curve_trans2_ccw = torch.cat([curve_trans2_ccw_x,curve_trans2_ccw_y],1)

#turn ccw
turn_ccw = torch.cat([curve_trans1_ccw,curve_turn_ccw,curve_trans2_ccw],0)


curve_trans1_cw_x = torch.zeros([int(math.acos((R2+d/2)/(R+R2))/w2)+2,1])
curve_trans1_cw_y = torch.zeros([int(math.acos((R2+d/2)/(R+R2))/w2)+2,1])

for i in range(0,int(math.acos((R2+d/2)/(R+R2))/w2)+2):
    curve_trans1_cw_x[i] =  R2*math.cos(math.pi/2+w2*i)
    curve_trans1_cw_y[i] = d - R2 + R2*math.sin(math.pi/2+w2*i)
curve_trans1_cw = torch.cat([curve_trans1_cw_x,curve_trans1_cw_y],1)

curve_turn_cw_x = torch.zeros([int((2*math.acos((R2+d/2)/(R+R2))+math.pi)/w)+2,1])
curve_turn_cw_y = torch.zeros([int((2*math.acos((R2+d/2)/(R+R2))+math.pi)/w)+2,1])

for i in range(0,int((2*math.acos((R2+d/2)/(R+R2))+math.pi)/w)+2):
    curve_turn_cw_x[i] = -math.sqrt((R2+R)**2 - (R2+d/2)**2) + R*math.cos(-w*i-math.pi/2+math.acos((R2+d/2)/(R+R2)))
    curve_turn_cw_y[i] = d/2 + d + R*math.sin(-w*i-math.pi/2+math.acos((R2+d/2)/(R+R2)))
curve_turn_cw = torch.cat([curve_turn_cw_x,curve_turn_cw_y],1)

curve_trans2_cw_x = torch.zeros([int(math.acos((R2+d/2)/(R+R2))/w2)+2,1])
curve_trans2_cw_y = torch.zeros([int(math.acos((R2+d/2)/(R+R2))/w2)+2,1])

for i in range(0,int(math.acos((R2+d/2)/(R+R2))/w2)+2):
    curve_trans2_cw_x[i] =  R2*math.cos(w2*i-math.pi/2-math.acos((R2+d/2)/(R+R2)))
    curve_trans2_cw_y[i] = 2*d + R2 + R2*math.sin(w2*i-math.pi/2-math.acos((R2+d/2)/(R+R2)))
curve_trans2_cw = torch.cat([curve_trans2_cw_x,curve_trans2_cw_y],1)
#turn ccw
turn_cw = torch.cat([curve_trans1_cw,curve_turn_cw,curve_trans2_cw],0)

path_profile2 = torch.cat([straight_forward,turn_ccw,straight_backward+torch.tensor([0,d]),turn_cw],0)

path_profile3 = torch.cat([path_profile2,path_profile2+torch.tensor([0,2*d]),path_profile2+torch.tensor([0,4*d]),\
                            path_profile2+torch.tensor([0,6*d]),path_profile2+torch.tensor([0,8*d]),path_profile2+torch.tensor([0,10*d]),\
                                path_profile2+torch.tensor([0,12*d]),path_profile2+torch.tensor([0,14*d]),path_profile2+torch.tensor([0,16*d])],0)

print(path_profile3.size())
                                

fig = plt.figure(num=2)
plt.plot(path_profile2[:,0],path_profile2[:,1])
plt.axis('equal')

fig = plt.figure(num=3)
plt.plot(path_profile3[:,0],path_profile3[:,1])
plt.axis('equal')
plt.show()
