#!/usr/bin/python3

import warnings

import numpy as np
import scipy

import util


class particlefilter:
    def __init__(self, count, start, posrange, angrange, polemeans, polevar, T_w_o=np.identity(4)):
        self.p_min = 0.01
        self.d_max = np.sqrt(-2.0 * polevar * np.log(np.sqrt(2.0 * np.pi * polevar) * self.p_min))
        self.minneff = 0.5
        self.estimatetype = 'best'
        self.count = count
        r = np.random.uniform(low=0.0, high=posrange, size=[self.count, 1])
        angle = np.random.uniform(low=-np.pi, high=np.pi, size=[self.count, 1])
        xy = r * np.hstack([np.cos(angle), np.sin(angle)])
        dxyp = np.hstack([xy, np.random.uniform(low=-angrange, high=angrange, size=[self.count, 1])])
        self.particles = np.matmul(start, util.xyp2ht(dxyp))
        self.weights = np.full(self.count, 1.0 / self.count)
        self.polemeans = polemeans # global map data
        self.poledist = scipy.stats.norm(loc=0.0, scale=np.sqrt(polevar))
        self.kdtree = scipy.spatial.cKDTree(polemeans[:, :2], leafsize=3)
        self.T_w_o = T_w_o
        self.T_o_w = util.invert_ht(self.T_w_o)
        self.Sigma = np.eye(3) * 1e-13

    @property
    def neff(self):
        return 1.0 / (np.sum(self.weights**2.0) * self.count)

    def update_motion(self, mean, cov):
        # mean, cov: odometry data [x, y, heading]
        T_r0_r1 = util.xyp2ht(np.random.multivariate_normal(mean, cov, self.count)) # SE(3) transformation of propagation

        self.particles = np.matmul(self.particles, T_r0_r1)

    def update_measurement(self, poleparams, resample=True):
        n = poleparams.shape[0] # number of poles(landmarks) received
        polepos_r = np.hstack([poleparams[:, :2], np.zeros([n, 1]), np.ones([n, 1])]).T # online poles(landmarks)
        for i in range(self.count):
            polepos_w = self.particles[i].dot(polepos_r)
            d, index = self.kdtree.query(polepos_w[:2].T, k = 1, distance_upper_bound = self.d_max)
            #delta = polepos_w[:2].T - self.polemeans[index, :2] # innovation
            #innov = np.hstack([delta, np.zeros([np.shape(delta)[0], 1]), np.zeros([np.shape(delta)[0], 1])]).T
            self.weights[i] *= np.prod(self.poledist.pdf(np.clip(d, 0.0, self.d_max)) + 0.1) # likelihood of measurement
        self.weights /= np.sum(self.weights)

        if resample and self.neff < self.minneff:
            self.resample()

    def estimate_pose(self):
        if self.estimatetype == 'mean':
            xyp = util.ht2xyp(np.matmul(self.T_o_w, self.particles))
            mean = np.hstack([np.average(xyp[:, :2], axis=0, weights=self.weights), util.average_angles(xyp[:, 2], weights=self.weights)])
            return self.T_w_o.dot(util.xyp2ht(mean))
        if self.estimatetype == 'max':
            return self.particles[np.argmax(self.weights)]
        if self.estimatetype == 'best':
            i = np.argsort(self.weights)[-int(0.1 * self.count):]
            xyp = util.ht2xyp(np.matmul(self.T_o_w, self.particles[i]))
            mean = np.hstack(
                [np.average(xyp[:, :2], axis=0, weights=self.weights[i]), util.average_angles(xyp[:, 2], weights=self.weights[i])])
            zero_mean_particles = xyp - mean
            zero_mean_particles[:,2] %= 2*np.pi   #wrap between 0 and 2pi
            self.Sigma = (1/xyp.shape[0])*zero_mean_particles.T @ zero_mean_particles
            return self.T_w_o.dot(util.xyp2ht(mean))

    def resample(self):
        cumsum = np.cumsum(self.weights)
        pos = np.random.rand() / self.count
        idx = np.empty(self.count, dtype=np.int)
        ics = 0
        for i in range(self.count):
            while cumsum[ics] < pos:
                ics += 1
            idx[i] = ics
            pos += 1.0 / self.count
        self.particles = self.particles[idx]
        self.weights[:] = 1.0 / self.count