import numpy as np

#EXAMPLE TO DETECT PERSON

class EKF:
    def set_F(self):
        F = np.eye(4)
        F[0,2] = F[1,3]  = self.dt
        return F
    
    def set_process_noise(self):
        G = np.zeros((4,2))
        G[0,0] = G[1,1] = np.power(self.dt,2)/2
        G[2,0] = G[3,1] = self.dt
        std_dev = np.eye(2)
        Q = G.dot(std_dev).dot(G.T) #assumed accelerations
        return Q

    def __init__(self, start_vector):
        self.dt = 0.1

        #State
        self.x = np.asarray(start_vector).reshape(4,1)
        self.P = np.eye(4)

        #State Transition
        self.F = self.set_F()

        #Noise
        self.Q = self.set_process_noise()

    def get_measurement(self):
        return np.ones(2).reshape(2,1)

    def get_HJacob(self, x_prime):
        #TO SIMPLIFY
        H = np.zeros((2,4))
        H[0,0] = H[1,1] = 1
        return H

    def predict(self):
        #prediction
        self.x_prime = np.dot(self.F,self.x).reshape(4,1)
        self.P_prime = self.F.dot(self.P).dot(self.F.T) + self.Q

    def update(self):
        #Measurement Update
        #LIDAR POSITION
        #RADAR SPEED
        #X Y Measurement
        z = self.get_measurement()

        #Measurement Matrix for KF
        #H = np.zeros((2,4))
        #H[0,0] = H[1,1] = 1

        #FOR EKF
        H = self.get_HJacob(self.x_prime)

        #Sensor Noise
        R = np.zeros((2,2))
        R[0,0] = R[1,1] = 0.01

        y = z - H.dot(self.x_prime)
        S = H.dot(self.P_prime).dot(H.T) + R
        K = self.P_prime.dot(H.T).dot(np.linalg.inv(S))
        self.x = self.x_prime + K.dot(y)
        I = np.eye(4)
        self.P = (I - K.dot(H)).dot(self.P_prime)
        print(self.x, self.P)


if __name__ == '__main__':
    px = 0
    py = 0
    vx = 0.1
    vy = 0

    start_vector = [px, py,vx, vy]
    ekf = EKF(start_vector)

    for i in range(3):
        ekf.predict()
        ekf.update()

        px = ekf.x[0]
        py = ekf.x[1]
        vx = ekf.x[2]
        vy = ekf.x[3]