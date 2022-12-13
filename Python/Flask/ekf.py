import numpy as np

#EXAMPLE TO DETECT PERSON

px = 0
py = 0
vx = 0.1
vy = 0

dt = 0.1

#State
x = np.asarray(([px, py,vx, vy])).reshape(4,1)
print(x.shape)
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
x_prime = np.dot(F,x).reshape(4,1)
print(x_prime.shape)
P_prime = F.dot(P).dot(F.T) + Q


#Measurement Update
#LIDAR POSITION
#RADAR SPEED
#X Y Measurement
z = np.ones(2).reshape(2,1)

#Measurement Matrix for KF
H = np.zeros((2,4))
H[0,0] = H[1,1] = 1

#FOR EKF
def get_HJacob(x_prime):
    #TO SIMPLIFY
    H = np.zeros((2,4))
    H[0,0] = H[1,1] = 1
    return H

H = get_HJacob(x_prime)

#Sensor Noise
R = np.zeros((2,2))
R[0,0] = R[1,1] = 0.01


y = z - H.dot(x_prime)
S = H.dot(P_prime).dot(H.T) + R
K = P_prime.dot(H.T).dot(np.linalg.inv(S))
x = x_prime + K.dot(y)
I = np.eye(4)
P = (I - K.dot(H)).dot(P_prime)

print(x.shape)
