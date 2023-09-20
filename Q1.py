import math as m
import numpy as np
import matplotlib.pyplot as plt
from spatialmath.base import *
from roboticstoolbox import ET2, DHRobot, RevoluteDH, PrismaticDH

def Space_Work(L1 = 1, L2 = 1):
    # Cria uma figura 3D
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')

    # Ângulos para o círculo em torno do eixo Z
    theta = np.linspace(0, np.pi, 100)
    theta2 = np.linspace(-np.pi/2, np.pi, 100)
    # Raio do círculo
    r = L1
    r2 = L1+L2

    # Coordenadas dos pontos no círculo
    x = r * np.cos(theta)
    y = r * np.sin(theta)
    z = np.zeros_like(theta)
    ax.plot(x, y, z, label='Junta 1')


    x1 = r2 * np.cos(theta2)
    y1 = r2 * np.sin(theta2)
    z1 = np.zeros_like(theta2)
    ax.plot(x1, y1, z1, label='Junta 2')


    # Defina os limites dos eixos
    ax.set_xlim([-1.5, 1.5])
    ax.set_ylim([-1.5, 1.5])
    ax.set_zlim([-0.5, 0.5])

    # Adicione rótulos de eixo
    ax.set_xlabel('Eixo X')
    ax.set_ylabel('Eixo Y')
    ax.set_zlabel('Eixo Z')

    ax.legend()
    plt.show()

    # Intervalo de valores de q1 e q2
    q1_values = np.linspace(0, np.pi, 100)
    q2_values = np.linspace(-np.pi/2, np.pi, 100)

    # Inicialize listas para armazenar as coordenadas x e y
    x_coords = []
    y_coords = []

    # Calcule as coordenadas cartesianas para cada par de valores de q1 e q2
    for q1 in q1_values:
        for q2 in q2_values:
            x = L1 * np.cos(q1) + L2 * np.cos(q1 + q2)
            y = L1 * np.sin(q1) + L2 * np.sin(q1 + q2)
            x_coords.append(x)
            y_coords.append(y)

    # Plote o espaço de trabalho
    plt.figure(figsize=(8, 6))
    plt.scatter(x_coords, y_coords, s=1, c='b', marker='.')
    plt.xlabel('Coordenada X')
    plt.ylabel('Coordenada Y')
    plt.title('Espaço de Trabalho do Manipulador RR Planar')
    plt.grid(True)
    plt.axis('equal')  # Manter proporções iguais nos eixos x e y
    plt.show()

def robot_RR(q = [0,0]):
    PLOT = True
    PI = np.pi

    e1 = RevoluteDH(d = 0, a  = 1 )
    e2 = RevoluteDH(a = 1 )

    rob = DHRobot([e1,e2], name = 'RR')
    #print(rob)
    rob.teach(q)

def inkine_RR(T= transl(0.5,1,0), L1=1, L2=1):
    R = [[T[0][0], T[0][1], T[0][2]], [T[1][0], T[1][1], T[1][2]], [T[2][0], T[2][1], T[2][2]]]
    P = [T[0][3], T[1][3], T[2][3]]

    x = P[0]
    y = P[1]

    print("Pose:\n", T)

    cO2 = (m.pow(x, 2) + m.pow(y, 2) - m.pow(L1, 2) - m.pow(L2, 2)) / (2 * L1 * L2)
    
    # Verificar se a condição é possível
    if abs(cO2) > 1:
        print("Configuração não é possível. Fora dos limites de alcance.")
        return
    
    sO2 = m.sqrt(1 - m.pow(cO2, 2))  # Raiz positiva
    O2 = m.atan2(sO2, cO2)

    k1 = (L1 + L2 * cO2)
    k2 = (L2 * sO2)
    gamma = m.atan2(k2, k1)
    r = m.sqrt(m.pow(k1, 2) + m.pow(k2, 2))

    k1 = r * m.cos(gamma)
    k2 = r * m.sin(gamma)

    O1 = m.atan2(y, x) - m.atan2(k2, k1)
    
    # Aplicar restrições para limitar O1 e O2 aos intervalos desejados
    O1_min = 0  # Limite inferior para O1
    O1_max = m.pi  # Limite superior para O1
    O2_min = -m.pi / 2  # Limite inferior para O2
    O2_max = m.pi  # Limite superior para O2
    
    print('Possíveis soluções:')
    # Verificar se os ângulos estão dentro dos limites
    if O1_min <= O1 <= O1_max and O2_min <= O2 <= O2_max:
        print("Solução 1:")
        print("O1:", O1, "O2:", O2)
        robot_RR([O1, O2])


    sO2 = -m.sqrt(1 - m.pow(cO2, 2))  # Raiz negativa
    O2 = m.atan2(sO2, cO2)

    k1 = (L1 + L2 * cO2)
    k2 = (L2 * sO2)
    gamma = m.atan2(k2, k1)
    r = m.sqrt(m.pow(k1, 2) + m.pow(k2, 2))

    k1 = r * m.cos(gamma)
    k2 = r * m.sin(gamma)

    O1 = m.atan2(y, x) - m.atan2(k2, k1)

    if O1_min <= O1 <= O1_max and O2_min <= O2 <= O2_max:
        print("Solução 1:")
        print("O1:", O1, "O2:", O2)
        robot_RR([O1, O2])





def main():
    Space_Work()
    inkine_RR()
    T = transl(0,0.5,0)
    inkine_RR(T)