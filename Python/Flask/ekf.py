import numpy as np
import matplotlib.pyplot as plt
#EXAMPLE FOR XYZ

class EKF:
    def set_F(self):
        F = np.eye(6)
        F[0,3] = F[1,4] = F[2,5] = self.dt
        return F
    
    def set_process_noise(self):
        G = np.zeros((self.x.shape[0],3))
        G[0,0] = G[1,1] = G[2,2] = np.power(self.dt,2)/2
        G[3,0] = G[4,1] = G[5,2] = self.dt
        std_dev = np.eye(3)*0.01
        Q = G.dot(std_dev).dot(G.T) #assumed accelerations
        return Q

    def __init__(self, start_vector):
        self.dt = 0.1

        #State
        self.x = np.asarray(start_vector).reshape(-1,1)
        self.P = np.eye(self.x.shape[0])

        #State Transition
        self.F = self.set_F()

        #Noise
        self.Q = self.set_process_noise()

    def get_measurement(self):
        z = np.zeros(3).reshape(-1,1)
        z[0] = 200.0 
        z[1] = 200.0
        return z + np.random.rand()*0.01

    def get_HJacob(self, x_prime):
        #TO SIMPLIFY
        # z_dim, x_dim
        H = np.zeros((3,self.x.shape[0]))
        H[0,2] = self.dt 
        H[1,4] = self.dt
        H[2,5] = self.dt
        H[0,0] = 1#self.dt 
        H[1,1] = 1#self.dt
        H[2,2] = 1#self.dt
        return H

    def predict(self):
        #prediction
        self.x_prime = np.dot(self.F,self.x).reshape(-1,1)
        self.P_prime = self.F.dot(self.P).dot(self.F.T) + self.Q

    def update(self):
        #Measurement Update
        #LIDAR POSITION
        #RADAR SPEED
        #X Y Measurement
        z = self.get_measurement()
        z_dim = z.shape[0]
        #Measurement Matrix for KF
        #H = np.zeros((2,4))
        #H[0,0] = H[1,1] = 1

        #FOR EKF
        H = self.get_HJacob(self.x_prime)

        #Sensor Noise
        R = np.zeros((z_dim,z_dim))
        R[0,0] = R[1,1] = 0.01

        y = z - H.dot(self.x_prime)
        S = H.dot(self.P_prime).dot(H.T) + R
        K = self.P_prime.dot(H.T).dot(np.linalg.inv(S))
        self.x = self.x_prime + K.dot(y)

        I = np.eye(self.x.shape[0])
        self.P = (I - K.dot(H)).dot(self.P_prime)


if __name__ == '__main__':
    px = 0
    py = 0
    pz = 0
    vx = 0
    vy = 0.
    vz = 0

    pos = []
    ppx = ppy = 0

    start_vector = [px, py, pz, vx, vy, vz]
    ekf = EKF(start_vector)
    pos.append([px,py])

    for i in range(3):
        ekf.predict()
        ekf.update()

        px = ekf.x[0]
        py = ekf.x[1]
        vx = ekf.x[2]
        vy = ekf.x[3]

        pos.append([px,py])
    
    pos = np.asarray(pos)
    plt.plot(pos[:,0], pos[:,1])
    plt.show()