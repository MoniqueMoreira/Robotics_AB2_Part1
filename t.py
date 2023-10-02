import roboticstoolbox as rtb
puma = rtb.models.DH.Puma560()                  # instantiate robot model
print(puma)
print(puma.qr)
T = puma.fkine([0.1, 0.2, 0.3, 0.4, 0.5, 0.6])  # forward kinematics
print(T)
sol = puma.ikine_LM(T)                          # inverse kinematics
print(sol)
puma.teach(sol.q)