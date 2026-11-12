

import numpy as np



# TODO Part 3: Comment the code explaining each part
"""
EKF implementation for the differential drive robot.
x is the state vector with robot pose (x,y,θ) and robot angular/linear speed
and lin acceleration (w, v, v_dot).
z is the measurement vector (odom v and w, imu ax and ay).
The predict/update steps run everytime a synced odom and imu pair arrives
"""
class kalman_filter:
    
    # TODO Part 3: Initialize the covariances and the states   
    """
    P: 6x6 state estimate covariance matrix, estimates uncertainty about current state
    Q: 6x6 noise covariance, estimates uncertainty in motion model, larger Q means EKF trust the motion model less
    R: 4x4 measurement covariance, estimates uncertainty with measurements, larger R means sensors are trusted less
    x: 6x1 state estimate (x,y,th,w,v,vdot)
    dt: time step between cycles 
    """ 
    def __init__(self, P, Q, R, x, dt):
        
        self.P= P
        self.Q= Q
        self.R= R
        self.x= x
        self.dt = dt
        
    # TODO Part 3: Replace the matrices with Jacobians where needed   
    """
    Prediction step: Estimates current robot state based on motion model
    """     
    def predict(self):

        # linearize motion model, evaluated at previous state
        self.A = self.jacobian_A()
        # predicts sensor output based on given state
        self.motion_model()
        # linearize measurement model, evaluated at predicted state
        self.C = self.jacobian_H()

        # calculate the covariance matrix, P uncertainty always increases during predict step
        self.P= np.dot( np.dot(self.A, self.P), self.A.T) + self.Q

    # TODO Part 3: Calculate the Kalman gain and correct your estimates
    """
    Update step: corrects covariance matrix with measurements from odom+imu vs predicted measurements
    """
    def update(self, z):

        # innovation covariance, expected uncertainty of what sensors should read + sensors noise
        s = np.dot(np.dot(self.C, self.P), self.C.T) + self.R

        # kalman gain, weighting between prediction and measurements, maps measurement error into state correction
        # how sensor measurement scorrect state estimation
        kalman_gain= np.dot(np.dot(self.P, self.C.T), np.linalg.inv(s))

        # actual measurements minus expected measurements
        error = np.array(z) - self.measurement_model()

        # state correction, moving predicted state towards measurement using kalman gain
        self.x= self.x + np.dot(kalman_gain, error)

        # covariance matrix update, measurements reduce uncertainty, P decreases
        self.P= np.dot(np.eye(self.A.shape[0]) - np.dot(kalman_gain, self.C), self.P)
        
    
    # TODO Part 3: Implement here the measurement model
    """
    Measurement model, predicts what sensors should read given a state.
    Used for innovation covariance matrix S
    """
    def measurement_model(self):
        x, y, th, w, v, vdot = self.x
        return np.array([
            v,# v from odom
            w,# w from odom
            vdot, # ax from imu
            v*w, # ay from imu
        ])
        
    # TODO Part 3: Impelment the motion model (state-transition matrice)
    """
    Motion model, moves state forward by one time step dt using kinematics equations.
    Predicts next state based on previous state.
    Assumes w and vdot are constant.
    """
    def motion_model(self):
        
        x, y, th, w, v, vdot = self.x
        dt = self.dt
        
        self.x = np.array([
            x +  v * np.cos(th) * dt,
            y + v * np.sin(th) * dt,
            th + w * dt,
            w,
            v  + vdot*dt,
            vdot,
        ])
        


    """
    Jacobian of motion model, used to update covariance matrix P,
    Derived from partial derivatives of motion model with respect to each state
    """
    def jacobian_A(self):
        x, y, th, w, v, vdot = self.x
        dt = self.dt
        
        return np.array([
            #x, y,               th, w,             v, vdot
            [1, 0,                -v * np.sin(th) * dt, 0,          np.cos(th) * dt,  0],
            [0, 1,                v * np.cos(th) * dt, 0,           np.sin(th) * dt,  0],
            [0, 0,                1, dt,                                          0,  0],
            [0, 0,                0, 1,                                           0,  0],
            [0, 0,                0, 0,                                          1,  dt],
            [0, 0,                0, 0,                                          0,  1 ]
        ])
    
    
    # TODO Part 3: Implement here the jacobian of the H matrix (measurements)    
    """
    Jacobian of measurement model, used to update S and K.
    Derived from partial derivatives of measurement model with respect to each state
    """
    def jacobian_H(self):
        x, y, th, w, v, vdot=self.x
        return np.array([
            #x, y,th, w, v,vdot
            [0,0,0  , 0, 1, 0], # h_v=v -> dh/dv=1
            [0,0,0  , 1, 0, 0], # h_w=w -> dh/dw=1
            [0,0,0  , 0, 0, 1], # h_ax=vdot -> dh/dvdot=1
            [0,0,0  , v, w, 0], # h_ay=v*w -> dh/dw=v, dh/dv=w
        ])
        
    # TODO Part 3: return the states here    
    # returns current state estimation, called after update()
    def get_states(self):
        return self.x
