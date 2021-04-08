#!/usr/bin/python3

import warnings
import numpy as np
import scipy
import util


class inEKF:
    #def __init__(self, count, start, posrange, angrange, 
    #        polemeans, polevar, T_w_o=np.identity(4)):
        
        
    def __init__(self):
        # InEKF Construct an instance of this class
        #
        # Input:
        #   system:     system and noise models
        sys = {}
        # motion model noise covariance
        sys['Q'] = np.diag(np.power([0.01, 0.01, 0.01], 2))
        sys['A'] = np.eye(3)
        # A is idenity comes from the fact the state transton is idenity
        sys['f'] = motion_model
        sys['H'] = measurement_error_matrix
        sys['N'] = np.diag(np.power([0.01, 0.01, 0.01], 2))
        
        self.A = system['A']  # error dynamics matrix
        self.f = system['f']  # process model
        self.H = system['H']  # measurement error matrix
        # Note that measurement error matrix is a constant for this problem, cuz gravity duhh
        self.Q = system['Q']  # input noise covariance
        self.N = system['N']  # measurement noise covariance
        self.X = np.eye(3)  # state vector
        self.P = 0.1 * np.eye(3)  # state covariance
        
        self.gfun = sys.gfun
        self.mu = init.mu
        self.Sigma = init.Sigma
        self.W = sys.W
        self.V = sys.V

        self.polemeans = polemeans # global map data
        self.poledist = scipy.stats.norm(loc=0.0, scale=np.sqrt(polevar))
        self.kdtree = scipy.spatial.cKDTree(polemeans[:, :2], leafsize=3)

    @property
    def Ad(self, X):
        # Adjoint of SO3 Adjoint (R) = R
        return X

    def skew(self,x):
        # This is useful for the rotation matrix
        #"""
        #:param x: Vector
        #:return: R in skew form (NOT R_transpose)
        #"""
        # vector to skew R^3 -> so(3)
        matrix = np.array([[0, -x[2], x[1]],
                           [x[2], 0, -x[0]],
                           [-x[1], x[0], 0]], dtype=float)
        return matrix
    
        
    def update_motion(self, u, Q):
        # Propagation model in SE(2)
        # u: relatve odometry data [x, y, heading], R^3
        # Q: noise covariance matrix, R^3x3
        T_rel = np.array([[np.cos(u[2]), -np.sin(u[2]), u[0]], [np.sin(u[2]), np.cos(u[2]), u[1]], [0, 0, 1]])
        self.mu_pred = self.mu @ T_rel

        AdjX = np.block([[self.mu[0:2, 0:2], np.array([self.mu[1,2], -self.mu[0,2]]).T], [0, 0, 1]]) # ERROR
        self.Sigma_pred = self.Sigma + AdjX @ Q @ AdjX.T
    
    
    def update_measurement(self, poleparams):
        n = poleparams.shape[0] # number of poles(landmarks) received
        polepos_r = np.hstack([poleparams[:, :2], np.ones([n, 1])]).T # online poles(landmarks)
        polepos_w = self.mu_pred @ polepos_r
        d, index = self.kdtree.query(polepos_w[:2].T, k = 1, distance_upper_bound = self.d_max)
        # len(index) = n: number of poles detected
        H = np.zeros((n * 2, 3)) # H matrix: 2n x 3
        v = np.zeros((n * 2, 1)) # innovation vector: 2n x 1
        # stack H matrix & innovation vector
        for i in range(n):
            H[2 * i, :] = [self.polemeans[index[i], 1], -1, 0]
            H[2 * i + 1, :] =  [-self.polemeans[index[i], 0], 0, -1]
            v[(2*i):(2*i+1)] = polepos_w[0:2, i] - self.polemeans[index[i], 0:2].T

        N_temp = self.mu_pred @ scipy.linalg.block_diag(self.V, 0) @ self.mu_pred.T # 3 x 3
        N = N_temp[0:2, 0:2]
        for i in range(n - 1):
            N = scipy.linalg.block_diag(N, N_temp[0:2, 0:2])        
        # N: 2n x 2n block-diagonal matrix
        S = H @ self.Sigma_pred @ H.T + N # S: 2n x 2n
        L = self.Sigma_pred @ H.T @ np.linalg.inv(S); # L: 3 x 2n
            
        # Update State
        self.mu = scipy.linalg.expm(wedge(L @ v)) @ self.mu_pred
            
        # Update Covariance
        self.Sigma = (np.identity(3) - L @ H) @ self.Sigma_pred @ (np.identity(3) - L @ H).T + L @ N @ L.T

def xhat = wedge(x)
    # wedge operation: convert an R^3 vector to Lie algebra se(2) ????
    G1 = np.array([[0, -1, 0], [1, 0, 0], [0, 0, 0]])
    G2 = np.array([[0, 0, 1], [0, 0, 0], [0, 0, 0]])
    G3 = np.array([[0, 0, 0], [0, 0, 1], [0, 0, 0]])
    xhat = G1 * x[0] + G2 * x[1] + G3 * x[2]
    return xhat
