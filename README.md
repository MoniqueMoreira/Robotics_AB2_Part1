﻿# Robotics_AB2_Part1

Importando as bibliotecas:

```
import math as m
import numpy as np
from roboticstoolbox DHRobot, RevoluteDH, PrismaticDH
import matplotlib.pyplot as plt
from spatialmath.base import *
```

## Questão 1:

### Letra A:
Sendo o espaço de trabalho do braço robô RR planar delimitado pelo comprimento dos elos L1 e L2, bem como pelas limitações de rotação das juntas, podemos visualizar a região da seguinte forma, considerando as restrições impostas: 
```
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

    # Adicione rótulos de eixo
    ax.axis('equal')
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
```
**Saída:**

<div style="display: flex;">
  <a name="figura1"></a>
  <img src="Figure_1.png" alt="q+" style="width: 47%;">
  <a name="figura2"></a>
  <img src="Figure_2.png" alt="" style="width: 45%;">
</div>

Podemos ver que, como o braço é planar, ele está restrito ao plano XY, onde na figura 1 vemos a restrições da juntas e na figura 2 podemos ver os pontos que podem ser alcançados definindo **L1 = L2 = 1**, e as juntas com rotação de **$` 0 < \theta1 < \pi`$  e $` -\pi/2 < \theta2 < \pi`$**. 

### Letra B:
Sabendo que a tabela DH do braço RR planas é:

| j | θⱼ  |  dⱼ |   aⱼ   | ⍺ⱼ   |
| --|-----|-----|--------|------|
| 1 | q1  |  0  |   L1   | 0.0° |
| 2 | q2  |  0  |   L2   | 0.0° |

Podemos calcular a Pose final da base até o atuador pela transformações:

$`^jT_{j+1}=\begin{bmatrix}\cos \theta _j&-\sin \theta _j\cos \alpha _j&\sin \theta _j\sin \alpha _j&a_j\cos \theta _j\\
\sin \theta _j&\cos \theta _j\cos \alpha _j&-\cos \theta _j\sin \alpha _j&a_j\sin \theta _j\\
0&\sin \alpha _j&\cos \alpha _j&d_j\\
0&0&0&1\end{bmatrix}`$

Onde a Pose final e dada por, no qual **$`S_{\theta1} = Sen(\theta1), C_{\theta1} = Cos(\theta1)`$** e assim por diante temos para braço acima:

$`^BT_W=^0T_2=^0T_1\cdot^1T_2=\begin{bmatrix}C_{\theta1}&-S_{\theta1}&0&L_1C_{\theta1}\\
S_{\theta1}&C_{\theta1}&0&L_1S_1\\
0&0&1&0\\
0&0&0&1\end{bmatrix}\begin{bmatrix}C_{\theta2}&-S_{\theta2}&0&L_2C_{\theta2}\\
S_{\theta2}&C_{\theta2}&0&L_2S_2\\
0&0&1&0\\
0&0&0&1\end{bmatrix}=\ \begin{bmatrix}C_{\theta1}C_{\theta2}&S_{\theta1}S_{\theta2}&0&L_1C_{\theta1}+L_2C_{\theta1}C_{\theta2}\\
S_{\theta1}S_{\theta2}&C_{\theta1}C_{\theta2}&0&L_1S_{\theta1}+L_2S_{\theta1}S_{\theta2}\\
0&0&1&0\\
0&0&0&1\end{bmatrix}`$

Para um Pose qualquer, dentro do espaço de trabalho:

$`^BT_W=\begin{bmatrix}C_{\phi}&-S_{\phi}&0&x\\
S_{\phi}&C_{\phi}&0&y\\
0&0&1&0\\
0&0&0&1\end{bmatrix}`$

Sendo x e y,  o ponto no plano XY que o atuador está localizado, igualando as poses obtemos as equações:

I. $`S_{\phi}=S_{\theta1}S_{\theta2}`$

II. $`C_{\phi}=C_{\theta1}C_{\theta2}`$

III. $`x=L_1C_{\theta1}+L_2C_{\theta1}C_{\theta2}`$

IV. $`y=L_1S_{\theta1}+L_2S_{\theta1}S_{\theta2}`$




### Letra C:

### Letra D:
