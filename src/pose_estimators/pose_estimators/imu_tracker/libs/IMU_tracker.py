import numpy as np
import mathlib as ml

class IMUTracker:
    def __init__(self, sample_rate:int) -> None:
        self.sample_rate = sample_rate
        self.initialized = False

        self.velocity = np.zeros(3, dtype=np.float32)
        self.position = np.zeros(3, dtype=np.float32)
        # self.orientation = np.zeros(4, dtype=np.float32) # quaternion

        self.prev_data_ts = 0

    def initialize(self, callib_data:np.ndarray) -> None:
        self.callib_data = callib_data
        self.callib_data_mean = np.mean(callib_data, axis=0)
        self.callib_data_std = np.std(callib_data, axis=0)
        self.initialized = True

    def calculatePose(self, imu_data:np.ndarray, data_ts:int) -> np.ndarray:
        if not self.initialized:
            raise Exception("Tracker not initialized.")
        
        delta_t = (data_ts - self.prev_data_ts) / 1e9

        # normalize data
        # imu_data_norm = (imu_data - self.callib_data_mean) / self.callib_data_std

        # high pass filter
        imu_data_filt = ml.filtSignal(imu_data, dt=delta_t, wn=self.callib_data_mean, btype='highpass', order=1)

        # calculate velocity
        self.velocity += imu_data_filt[0:3] * delta_t

        # calculate position
        self.position += self.velocity * delta_t

        self.prev_data_ts = data_ts

        return self.position