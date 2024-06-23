import torch
import math
import torch.nn as nn

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