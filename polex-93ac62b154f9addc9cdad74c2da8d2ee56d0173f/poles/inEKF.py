#!/usr/bin/python3

import warnings
import numpy as np
import scipy
import util
from sklearn.neighbors import BallTree, KDTree, DistanceMetric

class inEKF:
    def __init__(self, start, polemeans, polevar, T_w_o = np.identity(4)):
        start = np.block([[start[0:2, 0:2], start[0:2, 3:4]], [0, 0, 1]])
        self.mu = start
        self.Sigma = np.identity(3) * 1e-14

        self.polemeans = polemeans # global map data
        
        self.polevar = polevar
 
        # TO DO: tune this parameter:
        self.V = np.diag([polevar, polevar])

        dist_metric = DistanceMetric.get_metric('mahalanobis', V = self.V)
        self.balltree = BallTree(polemeans[:, :2], leafsize = 3, metric = dist_metric)
        
    def update_motion(self, u, cov):
        # Propagation model in SE(2)
        # u: relatve odometry data [x, y, heading], R^3
        # cov: noise covariance fo [x, y, heading], R^3x3

        # TO DO: tune this amplification constant
        cov = cov * 4
        Q = np.array([[cov[2,2], cov[2,0], cov[2, 1]],\
                      [cov[0,2], cov[0,0], cov[0, 1]],\
                      [cov[1,2], cov[1,0], cov[1, 1]]])
        
        # predict covariance
        Adj = np.block([[1, 0, 0], [np.array([[self.mu[1,2]], [-self.mu[0,2]]]), self.mu[0:2, 0:2]]])
        self.Sigma = self.Sigma + Adj @ Q @ Adj.T

        # predict state
        T_rel = xyp2SE2(u)
        self.mu = self.mu @ T_rel
    
    def update_measurement(self, poleparams):
        # poleparams: online poles(landmarks) detected by sensor 
        n = poleparams.shape[0] # number of poles(landmarks) detected
        polepos_r = np.hstack([poleparams[:, 0:2], np.ones((n, 1))]).T 
        polepos_w = self.mu @ polepos_r
        # data association using BallTree
        d, index = self.balltree.query(polepos_w[:2].T, k = 1)

        # stack H matrix & innovation vector
        H = []
        v = []
        neff_pole = 0
        for i in range(n):
            # Important: Accept poles with Mahalanobis distance < 3 (3-sigma)
            if d[i] <= np.sqrt(9.21):
                H.append([self.polemeans[index[i], 1][0], -1, 0])
                H.append([-self.polemeans[index[i], 0][0], 0, -1])
                delta = polepos_w[0:2, i] - [self.polemeans[index[i], 0][0], self.polemeans[index[i], 1][0]]
                v.append(delta[0])
                v.append(delta[1])
                neff_pole += 1

        if neff_pole > 0:
            H = np.array(H).reshape(-1, 3)
            v = np.array(v).reshape(-1, 1)
            
            N_temp = self.mu @ scipy.linalg.block_diag(self.V, 0) @ self.mu.T # 3 x 3
            N = N_temp[0:2, 0:2]
            for i in range(neff_pole - 1):
                N = scipy.linalg.block_diag(N, N_temp[0:2, 0:2])
            # N: 2neff x 2neff block-diagonal matrix
            S = H @ self.Sigma @ H.T + N # S: 2neff x 2neff
            L = self.Sigma @ H.T @ np.linalg.inv(S); # L: 3 x 2neff
    
            # Update State
            self.mu = scipy.linalg.expm(wedge(L @ v)) @ self.mu
            # Update Covariance
            self.Sigma = (np.identity(3) - L @ H) @ self.Sigma @ (np.identity(3) - L @ H).T + L @ N @ L.T

        #else:
            #print("neff_pole = ", neff_pole)

    def estimate_pose(self):
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
