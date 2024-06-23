import torch
import math
import numpy as np
from matplotlib import pyplot as plt
import torch.nn as nn



def coordinate2angle(p1,p2):
        x1,y1 = p1[:2]
        x2,y2 = p2[:2]
        dx = x2-x1
        dy = y2-y1
        ds = math.sqrt(dx**2 + dy**2)
        theta = math.acos(dx/ds)
        if dy<0:
            theta = -theta
        return theta
class Bezier(nn.Module):
    def __init__(self) -> None:
        super(Bezier,self).__init__()
        self.u = torch.arange(0.005,1,0.005)
        self.B = torch.zeros(4,self.u.shape[0])
        self.dB = torch.zeros(4,self.u.shape[0])
        self.ddB = torch.zeros(4,self.u.shape[0])
        self.P0 = None
        self.P1 = None
        self.P2 = None
        self.P3 = None

        for i in range(4):
                for j in range(self.u.shape[0]):
                        self.B[i,j] = math.factorial(3)/(math.factorial(i)*math.factorial(3-i))*(self.u[j]**i)*(1-self.u[j])**(3-i)
                        self.dB[i,j] = math.factorial(3)/(math.factorial(i)*math.factorial(3-i))*\
                                         (i*(self.u[j])**(i-1)*(1-self.u[j])**(3-i)-(self.u[j]**i)*(3-i)*(1-self.u[j])**(2-i))
                        self.ddB[i,j] = math.factorial(3)/(math.factorial(i)*math.factorial(3-i))*\
                                          (i*(i-1)*self.u[j]**(i-2)*(1-self.u[j])**(3-i)-i*(3-i)*self.u[j]**(i-1)*(1-self.u[j])**(2-i)-\
                                           (i*(3-i)*self.u[j]**(i-1)*(1-self.u[j])**(2-i))-self.u[j]**i*(3-i)*(2-i)*(1-self.u[j])**(1-i))
        
    def path_length(self,Curve):
        s = 0
        for i in range(Curve.shape[1]-1):
             s += math.sqrt((Curve[0,i+1]-Curve[0,i])**2 + (Curve[1,i+1]-Curve[1,i])**2)
        return s
    
    def mean_kappa(self,kappa_arr):
        return torch.mean(kappa_arr)

    def max_kappa(self,kappa_arr):
         return torch.max(kappa_arr)

    def kappa(self):
        dCurve = torch.kron(self.P0,self.dB[0,:]).reshape([2,-1]) +\
                                torch.kron(self.P1,self.dB[1,:]).reshape([2,-1]) +\
                                        torch.kron(self.P2,self.dB[2,:]).reshape([2,-1])+\
                                                torch.kron(self.P3,self.dB[3,:]).reshape([2,-1])
        
        ddCurve = torch.kron(self.P0,self.ddB[0,:]).reshape([2,-1]) +\
                                torch.kron(self.P1,self.ddB[1,:]).reshape([2,-1]) +\
                                        torch.kron(self.P2,self.ddB[2,:]).reshape([2,-1])+\
                                                torch.kron(self.P3,self.ddB[3,:]).reshape([2,-1])
        x_dot = dCurve[0,:]
        y_dot = dCurve[1,:]
        x_ddot = ddCurve[0,:]
        y_ddot = ddCurve[1,:]
        kappa_arr = torch.zeros_like(x_dot)
        for i in range(kappa_arr.shape[0]):
            kappa_arr[i] = abs(x_ddot[i]*y_dot[i]-x_dot[i]*y_ddot[i])/((x_dot[i]**2+y_dot[i]**2)**(3/2))
        return kappa_arr
    
    def forward(self,P0,P1,ratio0,ratio1,P0_psi,P1_psi):
        dis = math.sqrt((P1[0]-P0[0])**2 + (P1[1]-P0[1])**2)
        P2 = P0 + ratio0*dis*torch.tensor([math.cos(P0_psi),math.sin(P0_psi)])
        P3 = P1 - ratio1*dis*torch.tensor([math.cos(P1_psi),math.sin(P1_psi)])
        self.P0 = P0
        self.P1 = P2
        self.P2 = P3
        self.P3 = P1
        return self.curve(P0,P2,P3,P1)

    def curve(self,P0,P1,P2,P3):
        Curve = torch.kron(P0,self.B[0,:]).reshape([2,-1]) +\
                                torch.kron(P1,self.B[1,:]).reshape([2,-1]) +\
                                        torch.kron(P2,self.B[2,:]).reshape([2,-1])+\
                                                torch.kron(P3,self.B[3,:]).reshape([2,-1])
        return Curve
    
    


#parmas

init_len = 5 #m

#after verifying that receiving state estimation data
        #calculate max side
        #calculate angle between long side and east axis
p_way_point = torch.tensor([1.0,-1.0,0,5.0,10,0,-16.0,6.0,0,-20.0,2,0]).reshape([-1,3])

dist_arr = torch.zeros([p_way_point.shape[0]])
for index in range(p_way_point.shape[0]-1):
    dist_arr[index] = math.sqrt((p_way_point[index+1,0]-p_way_point[index,0])**2 +\
                                    (p_way_point[index+1,1]-p_way_point[index,1])**2)
dist_arr[p_way_point.shape[0]-1] = math.sqrt((p_way_point[-1,0]-p_way_point[0,0])**2 +\
                                    (p_way_point[-1,1]-p_way_point[0,1])**2)
print(torch.argmax(dist_arr))
max_len_num = torch.argmax(dist_arr)
if max_len_num != p_way_point.shape[0]-1:
    theta = coordinate2angle(p_way_point[max_len_num,:2],p_way_point[max_len_num+1,:2])
else:
    theta = coordinate2angle(p_way_point[-1,:2],p_way_point[0,:2])
theta2 = math.pi+theta
theta_arr = [theta,theta2]

psi0 = math.pi*1/7

start_pt = torch.tensor([init_len*math.cos(psi0),init_len*math.sin(psi0)])
curve = torch.zeros([p_way_point.shape[0],2,2,199])
curve_flatten = curve.reshape([p_way_point.shape[0]*2,2,199])
score = torch.zeros([p_way_point.shape[0]*2])
for i in range(p_way_point.shape[0]):
     for dir in range(2):
        bezier = Bezier()
        Curve = bezier(start_pt,p_way_point[i,:2],1/3,1/3,psi0,theta_arr[dir])
        curve[i,dir,:,:] = Curve
        curve_flatten[2*i+dir,:,:] = Curve
        kappa_arr = bezier.kappa()
        mean_kappa = bezier.mean_kappa(kappa_arr)
        max_kappa = bezier.max_kappa(kappa_arr)
        path_len = bezier.path_length(Curve)
        score[2*i+dir] = max_kappa*50 + mean_kappa*30 + path_len
        print('max_kappa: ',max_kappa)
        print('path_len: ',path_len)
        print('score: ',score[2*i+dir])

min_index = torch.argmin(score)
print(min_index)

print(p_way_point[0,:])
p_way_point = torch.concat([p_way_point,p_way_point[0,:].reshape([1,-1])])
#curve_flatten = curve.reshape([p_way_point.shape[0]*2,2,199])

plt.figure(num=1)
for i in range(curve.shape[0]):
     for j in range(curve.shape[1]):
          plt.plot(curve[i,j,0,:],curve[i,j,1,:])
plt.plot(curve_flatten[min_index,0,:],curve_flatten[min_index,1,:],linewidth=4)
plt.plot(p_way_point[:,0],p_way_point[:,1])
plt.axis('equal')
plt.show()