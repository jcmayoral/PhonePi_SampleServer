import numpy as np
import tkinter
from filterpy.kalman import ExtendedKalmanFilter
from filterpy.common import Q_discrete_white_noise

import time

class EKF(ExtendedKalmanFilter):
    def do_something_amazing(self,x):
        print(x)
        time.sleep(0.1)
   
    def calculate_jacobian(self,x):
        #print(np.array(x * np.random.rand()))
        #return np.array(x * np.random.rand()).T
        return np.eye(2) * np.random.rand()

    def predict_measure(self,z):
        return [z + np.random.rand()]

    def __init__(self):
        #state number of variables, measurements
        super().__init__(dim_x=2, dim_z=1, dim_u=1)
        self.x = np.array([[2.],
                        [0.]])       # initial state (location and velocity)

        self.F = np.array([[1.,1.],
                        [0.,1.]])    # state transition matrix

        self.H = np.array([[1.,0.]])    # Measurement function
        self.P *= 1000.                 # covariance matrix
        self.R = 5                      # state uncertainty
        self.Q = Q_discrete_white_noise(dim=2, dt=0.1, var=0.1) # process uncertainty



if __name__ == '__main__':
    my_filter = EKF()
    while True:
        my_filter.predict()
        my_filter.update(np.random.rand(1), my_filter.calculate_jacobian, my_filter.predict_measure)

        # do something with the output
        x = my_filter.x
        print(x)
        my_filter.do_something_amazing(x)

