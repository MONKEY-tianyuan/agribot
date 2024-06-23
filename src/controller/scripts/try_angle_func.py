import torch
import math


p1 = torch.tensor([2,3,3.0])
p2 = torch.tensor([1,2,4.0])


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


print(coordinate2angle(p1,p2))