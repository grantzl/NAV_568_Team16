<<<<<<< HEAD
#!/usr/bin/env python

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
    
    def neff(self):
        return 1.0 / (np.sum(self.weights**2.0) * self.count)

    def prediction(self, u):
        state(1) = self.mu(1,3)
        state(2) = self.mu(2,3)
        state(3) = atan2(self.mu(2,1), self.mu(1,1))
        H_prev = self.posemat(state)
        state_pred = self.gfun(state, u)
        H_pred = self.posemat(state_pred)

        u_se2 = logm(H_prev \ H_pred)

        Adjoint = @(X) [X(1:2,1:2), [X(2,3); -X(1,3)]; 0 0 1]; #THIS STILL NEEDS CONVERSION
        AdjX = Adjoint(H_prev)

        self.propagation(u_se2, AdjX)
    
        
    def propagation(self, u, AdjX):
        # SE(2) propagation model; the input is u \in se(2) plus noise
        # propagate mean
        self.mu_pred = self.mu * expm(u)
        # propagate covariance
        self.Sigma_pred = self.Sigma + AdjX * self.W * np.transpose(AdjX)
    
    
    def correction(self, Y, Y2, id, id2):
        # This needs to be a loaded list of relevant landmarks
        # Passed in the "poleparams"
        global FIELDINFO        
        landmark_x = FIELDINFO.MARKER_X_POS(id)
        landmark_y = FIELDINFO.MARKER_Y_POS(id)    
        landmark_x2 = FIELDINFO.MARKER_X_POS(id2)
        landmark_y2 = FIELDINFO.MARKER_Y_POS(id2)
        
        # THIS NEEDS TO BE CONVERTED TO PYTHON ARRAYS

        G1 = [...
            0     0     1;
            0     0     0;
            0     0     0]

        G2 = [...
            0     0     0;
            0     0     1;
            0     0     0]

        G3 = [...
            0    -1     0;
            1     0     0;
            0     0     0]

        b = [landmark_x;landmark_y;1]
        b2 = [landmark_x2;landmark_y2;1]

        H = [...
            -1  0 landmark_y;
            0 -1 -landmark_x]
        H2 = [...
            -1  0 landmark_y2;
            0 -1 -landmark_x2]

        H = [H;H2]


        #N = blkdiag([10000 0 ;0 10000],[10000 0 ;0 10000])

        R = self.mu_pred(1:2,1:2)
        R = blkdiag(R, R)
        nu1 = self.mu_pred*np.transpose(Y) - b
        nu2 = self.mu_pred*np.transpose(Y2) - b2

        N = R * blkdiag(self.V,self.V) * np.transpose(R)


        S = H * self.Sigma_pred * np.transpose(H) + N
        K = self.Sigma_pred * np.transpose(H) * (S \ np.eye(np.size(S)))

        delta1 = K(1:3,1:2) *[eye(2),zeros(2,1)]* nu1
        delta2 = K(1:3,3:4) *[eye(2),zeros(2,1)]* nu2

        delta = delta1 + delta2
        self.mu = expm(delta(1) * G1 + delta(2) * G2 + delta(3) * G3)*self.mu_pred

        self.Sigma = (eye(3) - K * H) * self.Sigma_pred * np.transpose((eye(3) - K * H) + K * N * np.transpose(K)
    end
        
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
        
        if neff_pole == 0:
            # if no neghboring poles exists, do nothing
            pass

        elif neff_pole > 0:
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
>>>>>>> e04441744f0d598ba1fd7c3d60b29e30d46028c1
