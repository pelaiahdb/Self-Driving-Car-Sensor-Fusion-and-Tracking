# ---------------------------------------------------------------------
# Project "Track 3D-Objects Over Time"
# Copyright (C) 2020, Dr. Antje Muntzinger / Dr. Andreas Haja.
#
# Purpose of this file : Kalman filter class
#
# You should have received a copy of the Udacity license together with this program.
#
# https://www.udacity.com/course/self-driving-car-engineer-nanodegree--nd013
# ----------------------------------------------------------------------
#

# imports
import numpy as np

# add project directory to python path to enable relative imports
import os
import sys
PACKAGE_PARENT = '..'
SCRIPT_DIR = os.path.dirname(os.path.realpath(os.path.join(os.getcwd(), os.path.expanduser(__file__))))
sys.path.append(os.path.normpath(os.path.join(SCRIPT_DIR, PACKAGE_PARENT)))
import misc.params as params 

class Filter:
    '''Kalman filter class'''
    def __init__(self):
        pass

    def F(self):
        ############
        # TODO Step 1: implement and return system matrix F
        ############

        # State transition function is a matrix F or system matrix
        # xk = f(xk-1) + v = F(xk-1) + v where v ~ N(0,Q) is a zero mean process noise with Q covariance

        # get delta t
        dt = params.dt

        # we use the 6D system matrix F from Lesson 6.7 for State Prediction

        system_matrix = np.matrix([
                                [1, 0, 0, dt, 0, 0],
                                [0, 1, 0, 0, dt, 0],
                                [0, 0, 1, 0, 0, dt],
                                [0, 0, 0, 1, 0, 0],
                                [0, 0, 0, 0, 1, 0],
                                [0, 0, 0, 0, 0, 1]
                                ])

        return system_matrix
        
        ############
        # END student code
        ############ 

    def Q(self):
        ############
        # TODO Step 1: implement and return process noise covariance Q
        ############

        # Process Noise Covariance Q
        # Q=E[ννT]
        # Q(Δt)=∫0Δt​F(t)QF(t)Tdt
        # q is a design parameter and should be chosen depending on the expected maximum change in velocity.
        # For highly dynamic maneuvers, we could use a higher process noise,
        # e.g. q=(8ms2)2 would fit for emergency braking,
        # whereas for normal situations on a highway e.g. q=(3s2m​)2 could be an adequate choice.
        

        dt = params.dt
        q = params.q

        # For a 6D Q

        q1 = ((dt**3)/3) * q 
        q2 = ((dt**2)/2) * q 
        q3 = dt * q

        return np.matrix([
                        [q1, 0, 0, q2, 0, 0],
                        [0, q1, 0, 0, q2, 0],
                        [0, 0, q1, 0, 0, q2],
                        [q2, 0, 0, q3, 0, 0],
                        [0, q2, 0, 0, q3, 0],
                        [0, 0, q2, 0, 0, q3]
                        ])
        
        ############
        # END student code
        ############ 

    def predict(self, track):
        ############
        # TODO Step 1: predict state x and estimation error covariance P to next timestep, save x and P in track
        ############

        # Lecture notes:
        # predict state and estimation error covariance to next timestep
        # F = self.F()
        # x = F*x # state prediction
        # P = F*P*F.transpose() + self.Q() # covariance prediction

        F = self.F()
        Q = self.Q()

        track.set_x(F * track.x)
        track.set_P(F * track.P * F.transpose() + Q)

        # pass
        
        ############
        # END student code
        ############ 

    def update(self, track, meas):
        ############
        # TODO Step 1: update state x and covariance P with associated measurement, save x and P in track
        ############

        # Lecture notes:
        # update state and covariance with associated measurement
        # H = self.H() # measurement matrix
        # gamma = z - H*x # residual
        # S = H*P*H.transpose() + R # covariance of residual
        # K = P*H.transpose()*np.linalg.inv(S) # Kalman gain
        # x = x + K*gamma # state update
        # I = np.identity(self.dim_state)
        # P = (I - K*H) * P # covariance update

        x = track.x
        P = track.P

        # get linear measurement matrix. note that get_hx is non-linear
        H = meas.sensor.get_H(x)

        # solve gamma function and call it
        gamma = self.gamma(track, meas)

        # solve S function and call it
        S = self.S(track, meas, H)

        K = P*H.transpose()*np.linalg.inv(S)
        x = x + K*gamma
        I = np.identity(params.dim_state)
        P = (I - K*H) * P

        track.set_x(x)
        track.set_P(P)
        
        ############
        # END student code
        ############ 
        track.update_attributes(meas)
    
    def gamma(self, track, meas):
        ############
        # TODO Step 1: calculate and return residual gamma
        ############

        # gamma = z - H*x # residual
        x = track.x

        # use linear H
        H = meas.sensor.get_H(x)
        z = meas.z
        gamma = z - H*x

        return gamma
        
        ############
        # END student code
        ############ 

    def S(self, track, meas, H):
        ############
        # TODO Step 1: calculate and return covariance of residual S
        ############

        # Lecture notes:
        # S = H*P*H.transpose() + R # covariance of residual

        P = track.P
        R = meas.R #covariance R
        S = H * P * H.transpose() + R

        return S
        
        ############
        # END student code
        ############ 