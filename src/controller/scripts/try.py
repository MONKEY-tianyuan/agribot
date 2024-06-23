import torch
import math
import matplotlib.pyplot as plt
import numpy as np

a = np.loadtxt('/home/bikebot/AGB_CTRL_ws/src/controller/scripts/x_8curve.txt',dtype=np.float)
b = torch.zeros([13011],dtype=torch.float32)
b = torch.from_numpy(a).float()

print(b)

t = torch.range(0,150,0.0002)
print(t.size())
xd=torch.zeros([2,t.size()[0]])
xd_after = torch.zeros([2,t.size()[0]])
xd_dot = torch.zeros([2,t.size()[0]])
v_abs = torch.zeros_like(t)
R0 = 8
R1  =4
omega0 = 0.05
omega1 = 0.1

for i in range(xd.size()[1]):
    xd[0,i] = R1*math.sin(omega1*t[i])
    xd[1,i] = -R0 + R0*math.cos(omega0*t[i])
L = 0
index = 0
for i in range(xd.size()[1]-1):
    L += math.sqrt((xd[0,i+1]-xd[0,i])**2+(xd[1,i+1]-xd[1,i])**2)
    if L>0.009:
        xd_after[0,index] = xd[0,i]
        xd_after[1,index] = xd[1,i]
        L=0
        index += 1


Rot = torch.tensor([[math.cos(1),math.sin(-1)],
                   [math.sin(1),math.cos(1)]])
xd_after = Rot.mm(xd_after)
for i in range(xd_after.size()[1]-1):
    xd_dot[0,i] = (xd_after[0,i+1]-xd_after[0,i])/0.01
    xd_dot[1,i] = (xd_after[1,i+1]-xd_after[1,i])/0.01
    v_abs[i] = math.sqrt(xd_dot[0,i]**2 + xd_dot[1,i]**2)

non_zero_index = 0
for i in range(xd_dot.size()[1]):
    if xd_dot[0,i]!=0 or xd_dot[1,i]!=0:
        non_zero_index += 1

xd_dot = xd_dot[:,:(non_zero_index-2)]
v_abs = v_abs[:(non_zero_index-2)]

plt.cla()
plt.plot(xd_after[0,:], xd_after[1,:], ".r", label="course")
plt.axis('equal')

plt.figure()
plt.plot(v_abs)
plt.figure()
plt.plot(xd_dot[0,:],label='x')
plt.plot(xd_dot[1,:],label='y')
plt.legend()
plt.show()