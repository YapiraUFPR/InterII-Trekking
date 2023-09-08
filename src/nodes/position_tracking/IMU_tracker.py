import numpy as np
from mathlib import *

"""
Credits: https://github.com/LibofRelax/IMU-Position-Tracking
Modified by Gabriel Pontarolo
"""

class IMUTracker:

    def __init__(self, sampling, data_order={'w': 1, 'a': 2, 'm': 3}):
        '''
        @param sampling: sampling rate of the IMU, in Hz
        @param tinit: initialization time where the device is expected to be stay still, in second
        @param data_order: specify the order of data in the data array
        '''

        super().__init__()
        # ---- parameters ----
        self.sampling = sampling
        self.dt = 1 / sampling    # second
        self.dts = self.dt**2   # second squared
        self.data_order = data_order

        # ---- helpers ----
        idx = {1: [0, 3], 2: [3, 6], 3: [6, 9]}
        self._widx = idx[data_order['w']]
        self._aidx = idx[data_order['a']]
        self._midx = idx[data_order['m']]

        self._p = np.array([[0, 0, 0]], dtype='float64').T
        self._prevt = -1
        self._t = 0

    def initialize(self, callib_data, noise_coefficient={'w': 100, 'a': 100, 'm': 10}):
        '''
        Algorithm initialization
        
        @param data: (,9) ndarray
        @param cut: cut the first few data to avoid potential corrupted data
        @param noise_coefficient: sensor noise is determined by variance magnitude times this coefficient
        
        Return: a list of initialization values used by EKF algorithm: 
        (gn, g0, mn, gyro_noise, gyro_bias, acc_noise, mag_noise)
        '''

        # discard the first few readings
        # for some reason they might fluctuate a lot
        data = callib_data[5:30]
        w = data[self._widx[0]:self._widx[1]].T
        a = data[self._aidx[0]:self._aidx[1]].T
        m = data[self._midx[0]:self._midx[1]].T

        # ---- gravity ----
        gn = -a.mean(axis=0)
        gn = gn[np.newaxis]
        # save the initial magnitude of gravity
        g0 = np.linalg.norm(gn)

        # ---- magnetic field ----
        mn = m.mean(axis=0)
        # magnitude is not important
        mn = normalized(mn)[np.newaxis]

        # ---- compute noise covariance ----
        avar = a.var(axis=0)
        wvar = w.var(axis=0)
        mvar = m.var(axis=0)
        print('acc var: %s, norm: %s' % (avar, np.linalg.norm(avar)))
        print('ang var: %s, norm: %s' % (wvar, np.linalg.norm(wvar)))
        print('mag var: %s, norm: %s' % (mvar, np.linalg.norm(mvar)))

        # ---- define sensor noise ----
        gyro_noise = noise_coefficient['w'] * np.linalg.norm(wvar)
        gyro_bias = w.mean(axis=0)
        acc_noise = noise_coefficient['a'] * np.linalg.norm(avar)
        mag_noise = noise_coefficient['m'] * np.linalg.norm(mvar)

        self._init_list = (gn, g0, mn, gyro_noise, gyro_bias, acc_noise, mag_noise)

    def attitudeTrack(self, data):
        '''
        Removes gravity from acceleration data and transform it into navigation frame.
        Also tracks device's orientation.
        
        @param data: (9) ndarray
        @param list: initialization values for EKF algorithm: 
        (gn, g0, mn, gyro_noise, gyro_bias, acc_noise, mag_noise)

        Return: (acc, orientation)
        '''

        # ------------------------------- #
        # ---- Initialization ----
        # ------------------------------- #
        gn, g0, mn, gyro_noise, gyro_bias, acc_noise, mag_noise = self._init_list
        w = data[self._widx[0]:self._widx[1]] - gyro_bias
        a = data[self._aidx[0]:self._aidx[1]]
        m = data[self._midx[0]:self._midx[1]]
        # sample_number = np.shape(data)[0]

        # ---- data container ----
        a_nav = []
        orix = []
        oriy = []
        oriz = []

        # ---- states and covariance matrix ----
        P = 1e-10 * I(4)    # state covariance matrix
        q = np.array([[1, 0, 0, 0]]).T    # quaternion state
        init_ori = I(3)   # initial orientation

        # ------------------------------- #
        # ---- Extended Kalman Filter ----
        # ------------------------------- #

        # all vectors are column vectors

        # ------------------------------- #
        # ---- 0. Data Preparation ----
        # ------------------------------- #

        wt = w[np.newaxis].T
        at = a[np.newaxis].T
        mt = normalized(m[np.newaxis].T)

        # ------------------------------- #
        # ---- 1. Propagation ----
        # ------------------------------- #

        Ft = F(q, wt, self.dt)
        Gt = G(q)
        Q = (gyro_noise * self.dt)**2 * Gt @ Gt.T

        q = normalized(Ft @ q)
        P = Ft @ P @ Ft.T + Q

        # ------------------------------- #
        # ---- 2. Measurement Update ----
        # ------------------------------- #

        # Use normalized measurements to reduce error!

        # ---- acc and mag prediction ----
        print(-rotate(q))
        print(gn)
        pa = normalized(-rotate(q) @ gn.T)
        pm = normalized(rotate(q) @ mn.T)

        # ---- residual ----
        Eps = np.vstack((normalized(at), mt)) - np.vstack((pa, pm))

        # ---- sensor noise ----
        # R = internal error + external error
        Ra = [(acc_noise / np.linalg.norm(at))**2 + (1 - g0 / np.linalg.norm(at))**2] * 3
        Rm = [mag_noise**2] * 3
        R = np.diag(Ra + Rm)

        # ---- kalman gain ----
        Ht = H(q, gn, mn)
        S = Ht @ P @ Ht.T + R
        K = P @ Ht.T @ np.linalg.inv(S)

        # ---- actual update ----
        q = q + K @ Eps
        P = P - K @ Ht @ P

        # ------------------------------- #
        # ---- 3. Post Correction ----
        # ------------------------------- #

        q = normalized(q)
        P = 0.5 * (P + P.T)    # make sure P is symmertical

        # ------------------------------- #
        # ---- 4. other things ----
        # ------------------------------- #

        # ---- navigation frame acceleration ----
        conj = -I(4)
        conj[0, 0] = 1
        an = rotate(conj @ q) @ at + gn

        # ---- navigation frame orientation ----
        orin = rotate(conj @ q) @ init_ori

        # ---- saving data ----
        a_nav = an.T[0]
        orix = orin.T[0, :]
        oriy = orin.T[1, :]
        oriz = orin.T[2, :]

        return (a_nav, orix, oriy, oriz)

    def positionTrack(self, a_nav):
        '''
        Simple integration of acc data and velocity data.
        
        @param a_nav: acc data
        @param velocities: velocity data
        
        Return: 3D coordinates in navigation frame

        Modfied to store the previous iteration's position and add the current velocity to it
        '''

        at = a_nav[np.newaxis].T
        self._p += at * self.dts
        return self._p.T[0]
    
    def calculatePosition(self, data):
        a_nav, orix, oriy, oriz = self.attitudeTrack(data)
        return self.positionTrack(a_nav)