import numpy as np
import matplotlib.pyplot as plt

# Comprimentos dos elos
L1 = 1.0
L2 = 0.3

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