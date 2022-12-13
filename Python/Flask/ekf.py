import numpy as np

px = 0
py = 0
vx = 0.1
vy = 0

dt = 0.1

#State
x = np.asarray(([px, py,vx, vy]))
P = np.eye(4)

#State Transition
F = np.eye(4)
F[0,2] = F[1,3]  = dt

#Noise
G = np.zeros((4,2))
G[0,0] = G[1,1] = np.power(dt,2)/2
G[2,0] = G[3,1] = dt

std_dev = np.eye(2)
Q = G.dot(std_dev).dot(G.T) #assumed accelerations

#prediction
x_prime = np.dot(F,x)
print(x_prime)
P_prime = F.dot(P).dot(F.T) + Q
