import math as m
import numpy as np
from roboticstoolbox import ET2, DHRobot, RevoluteDH, PrismaticDH
import matplotlib.pyplot as plt
from spatialmath.base import *

PLOT = True
PI = np.pi
L1 = 1
L2 = 1


fig = plt.figure()
ax = fig.add_subplot(111,projection='3d')
ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_zlabel('Z')

ax.set_xlim([-1, 3])
ax.set_ylim([-1, 3])
ax.set_zlim([0, 3])

O = transl(0, 0, 0)
J1 = transl(0, 0,1) @ trotx(PI) @ trotz(PI)
J2 = transl(0, L1, 1) @ trotx(PI/2) @ trotz(PI/2)
J3 = transl(0,L1+L2, 1) @ trotx(PI/2) @ trotz(PI/2)

trplot(O, frame="O", color="k")
trplot(J1, frame="1", color="b")
trplot(J2, frame="2", color="r")
trplot(J3, frame="3")

plt.show()