<<<<<<< HEAD
#!/usr/bin/python3
=======
<<<<<<< HEAD
#!/usr/bin/env python
>>>>>>> twhsu-stanley-main

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
        self.T_w_o = T_w_o[0:3, 0:3]
        self.T_o_w = np.linalg.inv(self.T_w_o)
        
    def update_motion(self, u, Q):
        # Propagation model in SE(2)
        # u: relatve odometry data [x, y, heading], R^3
        # Q: noise covariance, R^3x3

        # predict covariance
        AdjX = np.block([[self.mu[0:2, 0:2], np.array([[self.mu[1,2]], [-self.mu[0,2]]])], [0, 0, 1]])
        self.Sigma = self.Sigma + AdjX @ Q @ AdjX.T

        # predict state
        T_rel = xyp2SE2(u)
        self.mu = self.mu @ T_rel
    
    def update_measurement(self, poleparams):
        # poleparams: online poles(landmarks) detected by sensor 
        n = poleparams.shape[0] # number of poles(landmarks) detected
        polepos_r = np.hstack([poleparams[:, 0:2], np.ones((n, 1))]).T 
        polepos_w = self.mu @ polepos_r
        d, index = self.kdtree.query(polepos_w[:2].T, k = 1, distance_upper_bound = self.d_max)
        # len(index) = n: number of poles detected

        # stack H matrix & innovation vector
        H = []
        v = []
        neff_pole = 0
        for i in range(n):
            if index[i] < self.kdtree.n: # If Missing neighbors, then index[i] = self.kdtree.n
                H.append([self.kdtree.data[index[i], 1], -1, 0])
                H.append([-self.kdtree.data[index[i], 0], 0, -1])
                delta = polepos_w[0:2, i] - self.kdtree.data[index[i], 0:2].T
                v.append(delta[0])
                v.append(delta[1])
                neff_pole += 1
        
<<<<<<< HEAD
        if neff_pole == 0:
            # if no neghboring poles exists, do nothing
            pass

        elif neff_pole > 0:
            H = np.array(H).reshape(-1, 3)
            v = np.array(v).reshape(-1, 1)

=======
    def H = posemat(self,state):
        x = state(1)
        y = state(2)
        h = state(3)
        % construct a SE(2) matrix element
        H = [...
            cos(h) -sin(h) x;
            sin(h)  cos(h) y;
                 0       0 1]
    end
=======
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
        self.V = np.diag([polevar*4, polevar*4])

        dist_metric = DistanceMetric.get_metric('mahalanobis', V = self.V)
        self.balltree = BallTree(polemeans[:, :2], leafsize = 3, metric = dist_metric)
        
        #self.T_w_o = T_w_o[0:3, 0:3]
        #self.T_o_w = np.linalg.inv(self.T_w_o)
        
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
        Adj = np.block([[self.mu[0:2, 0:2], np.array([[self.mu[1,2]], [-self.mu[0,2]]])], [0, 0, 1]])
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
            if d[i] <= 3:
                H.append([self.polemeans[index[i], 1][0], -1, 0])
                H.append([-self.polemeans[index[i], 0][0], 0, -1])
                delta = polepos_w[0:2, i] - [self.polemeans[index[i], 0][0], self.polemeans[index[i], 1][0]]
                v.append(delta[0])
                v.append(delta[1])
                neff_pole += 1

        if neff_pole > 0:
            H = np.array(H).reshape(-1, 3)
            v = np.array(v).reshape(-1, 1)
            
>>>>>>> twhsu-stanley-main
            N_temp = self.mu @ scipy.linalg.block_diag(self.V, 0) @ self.mu.T # 3 x 3
            N = N_temp[0:2, 0:2]
            for i in range(neff_pole - 1):
                N = scipy.linalg.block_diag(N, N_temp[0:2, 0:2])
            # N: 2neff x 2neff block-diagonal matrix
            S = H @ self.Sigma @ H.T + N # S: 2neff x 2neff
            L = self.Sigma @ H.T @ np.linalg.inv(S); # L: 3 x 2neff
<<<<<<< HEAD
                
=======
    
>>>>>>> twhsu-stanley-main
            # Update State
            self.mu = scipy.linalg.expm(wedge(L @ v)) @ self.mu
            # Update Covariance
            self.Sigma = (np.identity(3) - L @ H) @ self.Sigma @ (np.identity(3) - L @ H).T + L @ N @ L.T

<<<<<<< HEAD
    def estimate_pose(self):
        mu_SE3 = np.block([[scipy.linalg.block_diag(self.mu[0:2, 0:2], -1), np.array([[self.mu[0, 2]], [self.mu[1, 2]], [0]])],\
                               [0, 0, 0, 1]])
=======
        else:
            print("neff_pole = ", neff_pole)
    
    """
    def update_measurement_sequential(self, poleparams):
        # poleparams: online poles(landmarks) detected by sensor 
        n = poleparams.shape[0] # number of poles(landmarks) detected
        polepos_r = np.hstack([poleparams[:, 0:2], np.ones((n, 1))]).T 
        polepos_w = self.mu @ polepos_r
        d_max = np.sqrt(-2.0 * self.polevar * np.log(np.sqrt(2.0 * np.pi * self.polevar) * self.p_min))
        d, index = self.kdtree.query(polepos_w[:2].T, k = 1, distance_upper_bound = d_max)
        # len(index) = n: number of poles detected
        

        for i in range(n):
            if index[i] < self.kdtree.n:
                H = np.array([[self.kdtree.data[index[i], 1], -1, 0], [-self.kdtree.data[index[i], 0], 0, -1]]) # 2 x 3
                delta = polepos_w[0:2, i] - self.kdtree.data[index[i], 0:2].T
                v = np.array([[delta[0]], [delta[1]]])
                
                V = np.diag([100, 100])

                N_temp = self.mu @ scipy.linalg.block_diag(V, 0) @ self.mu.T # 3 x 3
                N = N_temp[0:2, 0:2]
                # N: 2neff x 2neff block-diagonal matrix
                S = H @ self.Sigma @ H.T + N # S: 2neff x 2neff
                L = self.Sigma @ H.T @ np.linalg.inv(S); # L: 3 x 2neff
                        
                # Update State
                self.mu = scipy.linalg.expm(wedge(L @ v)) @ self.mu
                # Update Covariance
                self.Sigma = (np.identity(3) - L @ H) @ self.Sigma @ (np.identity(3) - L @ H).T + L @ N @ L.T
    """

    def estimate_pose(self):
        mu_SE3 = np.block([[scipy.linalg.block_diag(self.mu[0:2, 0:2], -1), np.array([[self.mu[0, 2]], [self.mu[1, 2]], [0]])],\
                           [0, 0, 0, 1]])
>>>>>>> twhsu-stanley-main
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
<<<<<<< HEAD
=======
>>>>>>> 5bc42f22fa657326db4fb51acd1bc1eef99e606f
>>>>>>> twhsu-stanley-main
