import numpy as np
import tkinter
from filterpy.kalman import ExtendedKalmanFilter
from filterpy.common import Q_discrete_white_noise
import matplotlib.pyplot as plt
import time

class EKF(ExtendedKalmanFilter):
    def do_something_amazing(self):
        plt.plot(self.x, c="blue")
        plt.show()
        time.sleep(0.1)
   
    def calculate_jacobian(self,x):
        #print(np.array(x * np.random.rand()))
        #return np.array(x * np.random.rand()).T
        #J = np.zeros((2,1))
        J = np.eye(2)
        #J[0,1] = self.dt
        return J #np.random.rand()

    def Hx(self,x):
        #print ("PM", self.H.dot(z))
        Hx = np.eye(2)
        #print("HX" ,self.x - np.dot(self.K, self.y))
        #Hx[1] = 1
        return Hx#Hx.dot(x)

    def __init__(self):
        #state number of variables, measurements
        super().__init__(dim_x=2, dim_z=2, dim_u=1)
        self.i = 0
        self.dt = 0.1
        self.data = []

        self.B = np.eye(2)

        self.x = np.array(([2],[2])   # initial state (location and velocity)

        self.F = np.array([[1.,self.dt],
                        [0.,1.]])    # state transition matrix

        #self.H = np.array([[1.,0.]])    # Measurement function
        self.P *= 1.                 # covariance matrix
        self.R = 1                      # state uncertainty
        self.Q = Q_discrete_white_noise(dim=2, dt=0.1, var=0.1) # process uncertainty

    def get_measurement(self):
        self.i += 1
        return 1,1#[self.i, self.i]#np.random.rand()


if __name__ == '__main__':
    my_filter = EKF()
    while True:
        my_filter.predict()
        print("AF", my_filter.x)
        my_filter.update(None, my_filter.calculate_jacobian, my_filter.Hx)
        print("BF", my_filter.x)
        # do something with the output
        my_filter.do_something_amazing()

