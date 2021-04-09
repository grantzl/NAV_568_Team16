#!/usr/bin/python3

import warnings
import numpy as np
import scipy
import util

class inEKF:
    def __init__(self, start, posrange, angrange, polemeans, polevar, T_w_o = np.identity(4)):
        #r = np.random.uniform(low = 0.0, high = posrange)
        #angle = np.random.uniform(low = -np.pi, high = np.pi)
        #xy = r * np.hstack([np.cos(angle), np.sin(angle)])
        #dxyp = np.hstack([xy, np.random.uniform(low = -angrange, high = angrange)])
        start = np.block([[start[0:2, 0:2], start[0:2, 3:4]], [0, 0, 1]])
        self.mu = start #@ xyp2SE2(dxyp) # prior
        self.Sigma = np.identity(3) * 0.1

        self.V = np.diag([polevar/1.4, polevar/1.4]) # not sure if this is correct

        self.p_min = 0.01
        self.d_max = np.sqrt(-2.0 * polevar * np.log(np.sqrt(2.0 * np.pi * polevar) * self.p_min))
        self.polemeans = polemeans # global map data
        self.poledist = scipy.stats.norm(loc = 0.0, scale = np.sqrt(polevar))
        self.kdtree = scipy.spatial.cKDTree(polemeans[:, :2], leafsize = 3)
        self.T_w_o = T_w_o
        self.T_o_w = util.invert_ht(self.T_w_o)
        
    def update_motion(self, u, Q):
        # Propagation model in SE(2)
        # u: relatve odometry data [x, y, heading], R^3
        # Q: noise covariance, R^3x3

        # predict state
        T_rel = xyp2SE2(u)
        self.mu_pred = self.mu @ T_rel

        # predict covariance
        AdjX = np.block([[self.mu[0:2, 0:2], np.array([[self.mu[1,2]], [-self.mu[0,2]]])], [0, 0, 1]])
        self.Sigma_pred = self.Sigma + AdjX @ Q @ AdjX.T
    
    def update_measurement(self, poleparams):
        # poleparams: online poles(landmarks) detected by sensor 
        n = poleparams.shape[0] # number of poles(landmarks) detected
        polepos_r = np.hstack([poleparams[:, :2], np.ones([n, 1])]).T 
        polepos_w = self.mu_pred @ polepos_r
        d, index = self.kdtree.query(polepos_w[:2].T, k = 1, distance_upper_bound = self.d_max)
        # len(index) = n: number of poles detected

        # stack H matrix & innovation vector
        H = []
        v = []
        neff_pole = 0
        for i in range(n):
            # If Missing neighbors, then index[i] = self.kdtree.n
            if index[i] < self.kdtree.n:
                H.append([self.kdtree.data[index[i], 1], -1, 0])
                H.append([-self.kdtree.data[index[i], 0], 0, -1])
                delta = polepos_w[0:2, i] - self.kdtree.data[index[i], 0:2].T ##??
                v.append(delta[0])
                v.append(delta[1])

                neff_pole += 1
        
        if neff_pole == 0:
            # if no neghboring poles exists, do nothing
            self.mu = self.mu_pred
            self.Sigma = self.Sigma_pred
        else:

            H = np.array(H).reshape(-1, 3)
            v = np.array(v).reshape(-1, 1)
            
            N_temp = self.mu_pred @ scipy.linalg.block_diag(self.V, 0) @ self.mu_pred.T # 3 x 3
            N = N_temp[0:2, 0:2]
            for i in range(neff_pole - 1):
                N = scipy.linalg.block_diag(N, N_temp[0:2, 0:2])        
            # N: 2n x 2n block-diagonal matrix
            S = H @ self.Sigma_pred @ H.T + N # S: 2neff x 2neff
            L = self.Sigma_pred @ H.T @ np.linalg.inv(S); # L: 3 x 2neff
                
            # Update State
            self.mu = scipy.linalg.expm(wedge(L @ v)) @ self.mu_pred
        
            # Update Covariance
            self.Sigma = (np.identity(3) - L @ H) @ self.Sigma_pred @ (np.identity(3) - L @ H).T + L @ N @ L.T

    def estimate_pose(self):
        #xyp = util.ht2xyp(self.T_o_w @ self.mu)
        #result = self.T_w_o @ util.xyp2ht(xyp)

        # convert self.mu from SE(2) to SE(3)
        # not sure if this is correct
        mu_SE3 = np.block([[scipy.linalg.block_diag(self.mu[0:2, 0:2], -1), np.array([[self.mu[0, 2]], [self.mu[1, 2]], [0]])],\
                               [0, 0, 0, 1]])

        return mu_SE3

def wedge(x):
    # wedge operation: convert an R^3 vector to Lie algebra se(2) ????
    G1 = np.array([[0, -1, 0], [1, 0, 0], [0, 0, 0]])
    G2 = np.array([[0, 0, 1], [0, 0, 0], [0, 0, 0]])
    G3 = np.array([[0, 0, 0], [0, 0, 1], [0, 0, 0]])

    xhat = G1 * x[0] + G2 * x[1] + G3 * x[2]
    return xhat

def xyp2SE2(xyp):
    return np.array([[np.cos(xyp[2]), -np.sin(xyp[2]), xyp[0]], [np.sin(xyp[2]), np.cos(xyp[2]), xyp[1]], [0, 0, 1]])
