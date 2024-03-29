import math as m
import numpy as np
import matplotlib.pyplot as plt
from spatialmath.base import *
from roboticstoolbox import DHRobot, RevoluteDH, PrismaticDH

PI = np.pi

def Space_Work(L1 = 0.15,L2 = 0.15):
    # Cria uma figura 3D
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')

    # Ângulos para o círculo em torno do eixo Z
    theta = np.linspace(-np.pi/2, np.pi, 20)
    theta1 = np.linspace(-np.pi/2, np.pi, 20)
    theta2 = np.linspace(-np.pi/2, np.pi, 20)

    # Raio do círculos
    r = L1
    r1 = L1
    r2 = L1+L2

    # Coordenadas dos pontos no círculo
    x = r * np.cos(theta)
    y = r * np.sin(theta)
    z= np.zeros_like(theta)
    ax.plot(x, y, z, label='Junta 1')

    x1 = r1 * np.cos(theta1)
    y1 = np.zeros_like(theta1) 
    z1 = r1 * np.sin(theta1)
    ax.plot(x1, y1, z1, label='Junta 2')

    # Coordenadas dos pontos no círculo
    x2 = r2 * np.cos(theta2)
    y2= r2 * np.sin(theta2)
    z2= np.zeros_like(theta2)
    ax.plot(x2, y2, z2, label='Junta 3')

    # Adicione rótulos de eixo
    ax.axis('equal')
    ax.set_xlabel('Eixo X')
    ax.set_ylabel('Eixo Y')
    ax.set_zlabel('Eixo Z')

    ax.legend()
    plt.show()


    # Inicialize listas para armazenar as coordenadas x, y e z
    x_coords = []
    y_coords = []
    z_coords = []

    # Calcule as coordenadas cartesianas tridimensionais para cada conjunto de valores de q1, q2 e q3
    for q1 in theta:
        for q2 in theta1:
            for q3 in theta2:

                x = -L2*m.cos(q2)*m.sin(q1)-L1*m.sin(q3)*m.cos(q2)*m.sin(q1)-L1*m.cos(q3)*m.sin(q2)*m.sin(q1)
                y = L2*m.cos(q2)*m.cos(q1)+L1*m.sin(q3)*m.cos(q2)*m.cos(q1)+L1*m.cos(q3)*m.sin(q2)*m.cos(q1)
                z = L2*m.sin(q2)+L1*m.sin(q3)*m.sin(q2)-L1*m.cos(q3)*m.cos(q2)

                x_coords.append(x)
                y_coords.append(y)
                z_coords.append(z)

    # Plote o espaço de trabalho tridimensional
    fig = plt.figure(figsize=(8, 6))
    ax = fig.add_subplot(111, projection='3d')
    ax.scatter(x_coords, y_coords, z_coords, s=1, c='b', marker='.')
    ax.set_xlabel('Coordenada X')
    ax.set_ylabel('Coordenada Y')
    ax.set_zlabel('Coordenada Z')
    ax.set_title('Espaço de Trabalho do Manipulador RRR')
    plt.show()


def inkine_RRR(x, y, z, L1=0.15, L2=0.15):
    cont = 0
    Co = (y**2 + z**2 - L1**2 - L2**2)/(2*L1*L2)

    if abs(Co) > 1:
        print("-1 < Cos theta > 1")
        return None

    So = m.sqrt(1 - Co**2) # Raiz Positiva
    o3 = m.atan2(So, Co)

    So1 = -m.sqrt(1 - Co**2) # Raiz Negativa 
    o31 = m.atan2(So1, Co)

    b = m.atan2(z, y)

    r = m.sqrt(y**2 + z**2)
    if r == 0:
        Cp = 0
    else:
        Cp = (y**2 + z**2 + L1**2 - L2**2)/(2*L1*r)

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

    o1 = m.atan2(y, x)

    print("Possivéis soluções:")
    if abs(o1 - PI/2) > m.pi/2 or abs(o2) > m.pi/2 or abs(o3+PI/2) > m.pi/2:
        
        print('θ1=',o1- PI/2,'θ2=',o2,'θ3=',o3+PI/2)
        q = [o1 - PI/2,o2,o3 + PI/2]
        robot_RRR(q = q)
        cont= +1
    
    if abs(o1) > m.pi/2 or abs(o21) > m.pi/2 or abs(o31) > m.pi/2:
            print('θ1=',o1- PI/2,'θ2=',o21,'θ3=',o31+PI/2)
            q = [o1 - PI/2,o21,o31 + PI/2]
            robot_RRR(q = q)
            cont=+1
    if cont ==0:
        print("Não há solução dentro do intervalo -π/2 < θ1, θ2, θ3 < π/2")
        return None

def robot_RRR(q = [0,0,0],L1 = 0.15,L2 = 0.15):

    e1 = RevoluteDH(d = 0, alpha = PI/2, offset = PI/2)
    e2 = RevoluteDH(a = L1)
    e3 = RevoluteDH(a = L2,offset = -PI/2)

    rob = DHRobot([e1,e2,e3], name = 'RRR')
    #print(rob)
    rob.teach(q)
    return rob

def main():
    Space_Work()
    rob = robot_RRR()

    inkine_RRR(x=0, y=0.15,z =0)
    
    # Cinemática inversa
    P = rob.fkine(q = [0.0,-1.0471975511965976,3.6651914291880923])
    print('Pose =\n', P)
    sol = rob.ikine_LM(P)
    print('Solução da Biblioteca:\n',sol)
    
    P = rob.fkine(q = [0.0,1.0471975511965976,-0.5235987755982991])
    print('Pose =\n', P)
    sol = rob.ikine_LM(P)
    print('Solução da Biblioteca:\n',sol)
    
    
    inkine_RRR(x=0,y=0.07,z=-0.07)

    # Cinemática inversa
    P = rob.fkine(q = [0.0,-2.0199087495022043,4.039817499004409])
    print('Pose =\n', P)
    sol = rob.ikine_LM(P)
    print('Solução da Biblioteca:\n',sol)
    
    P = rob.fkine(q = [0.0,0.4491124227073078,-0.8982248454146156])
    print('Pose =\n', P)
    sol = rob.ikine_LM(P)
    print('Solução da Biblioteca:\n',sol)
    