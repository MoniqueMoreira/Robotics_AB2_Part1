import numpy as np
import matplotlib.pyplot as plt

def Q1(L1 = 1, L2 = 1):
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
