import math as m
import numpy as np
import matplotlib.pyplot as plt
from spatialmath.base import *
from roboticstoolbox import ET2, DHRobot, RevoluteDH, PrismaticDH
PLOT = True
PI = np.pi

def inkine_RRR(x, y, z, L1=1, L2=1):
    Co = (y**2 + z**2 - L1**2 - L2**2)/(2*L1*L2)

    if abs(Co) > 1:
        print("-1 < Cos theta > 1")
        return None

    So = m.sqrt(1 - Co**2) # Raiz Positiva
    o3 = m.atan2(So, Co)

    So1 = -m.sqrt(1 - Co**2) # Raiz Negativa 
    o31 = m.atan2(So1, Co)

    b = m.atan2(z, y)

    Cp = (y**2 + z**2 + L1**2 - L2**2)/(2*L1*m.sqrt(y**2 + z**2))
    if abs(Cp) > 1 :
        print("-1 < Cos phi > 1")
        return None

    Sp = m.sqrt(1 - Cp**2)
    p = m.atan2(Sp, Cp)


    if o3 > 0:
        o2 = b - p
        o21 = b + p
    else:
        o2 = b + p
        o21 = b - p

    o1 = m.atan2(y/m.sqrt(x**2 + y**2), x/m.sqrt(x**2 + y**2))

    print("Possivéis soluções:")
    if abs(o1 - PI/2) > m.pi/2 or abs(o2) > m.pi/2 or abs(o3+PI/2) > m.pi/2:
        print('θ1=',o1- PI/2,'θ2=',o2,'θ3=',o3+PI/2)
        q = [o1 - PI/2,o21,o31 + PI/2]
        robot_RRR(q = q)
        if abs(o1) > m.pi/2 or abs(o21) > m.pi/2 or abs(o31) > m.pi/2:
            print('θ1=',o1- PI/2,'θ2=',o21,'θ3=',o31+PI/2)
            q = [o1 - PI/2,o2,o3 + PI/2]
            robot_RRR(q = q)
    else:
        print("Não há solução dentro do intervalo -π/2 < θ1, θ2, θ3 < π/2")
        return None

def robot_RRR(q = [0,0,0],L1 = 1,L2 = 1):

    e1 = RevoluteDH(d = 0, alpha = PI/2, offset = PI/2)
    e2 = RevoluteDH(a = L1)
    e3 = RevoluteDH(a = L2,offset = -PI/2)

    rob = DHRobot([e1,e2,e3], name = 'RRR')
    #print(rob)
    rob.teach(q)
    return rob

def main():
    rob = robot_RRR()

    #inkine_RRR(x=0, y=1,z =0)
    print(rob.ikine_LM(transl(x=0, y=1,z =0)))
    rob = robot_RRR( q=[-0.1495, 0.9527, -0.6337])
    
    inkine_RRR(x=0,y=0.5,z=-0.5)
    print(rob.ikine_LM(transl(x=0,y=0.5,z=-0.5)))
    rob = robot_RRR(q=[-1.368, -2.779, 4.182])
    