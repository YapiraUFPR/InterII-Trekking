import numpy as np

class IMUTracker:
    def __init__(self, sample_rate:int) -> None:
        self.sample_rate = sample_rate
        self.initialized = False

        self.delta_t = 1 / sample_rate # time between samples in seconds for integration

        self.velocity = np.zeros(3, dtype=np.float32)
        self.position = np.zeros(3, dtype=np.float32)
        self.orientation = np.zeros(4, dtype=np.float32) # quaternion

    def initialize(self, callib_data:np.ndarray) -> None:
        self.callib_data = callib_data
        self.callib_data_mean = np.mean(callib_data, axis=0)
        self.callib_data_std = np.std(callib_data, axis=0)
        self.initialized = True

    def calculatePose(self, imu_data:np.ndarray) -> np.ndarray:
        if not self.initialized:
            raise Exception("Tracker not initialized.")
        
        # normalize data
        imu_data_norm = (imu_data - self.callib_data_mean) / self.callib_data_std

        # calculate orientation
        