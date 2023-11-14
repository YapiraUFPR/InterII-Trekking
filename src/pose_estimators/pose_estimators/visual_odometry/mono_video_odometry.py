import numpy as np
import cv2
import os

class MonoVideoOdometery(object):
    def __init__(self, 
                first_image,
                focal_length = 590.386253,
                pp = (318.077704, 233.468229), 
                lk_params=dict(winSize  = (21,21), criteria = (cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 30, 0.01)), 
            ):
        '''
        Arguments:
            img_file_path {str} -- File path that leads to image sequences
            pose_file_path {str} -- File path that leads to true poses from image sequence
        
        Keyword Arguments:
            focal_length {float} -- Focal length of camera used in image sequence (default: {718.8560})
            pp {tuple} -- Principal point of camera in image sequence (default: {(607.1928, 185.2157)})
            lk_params {dict} -- Parameters for Lucas Kanade optical flow (default: {dict(winSize  = (21,21), criteria = (cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 30, 0.01))})
            detector {cv2.FeatureDetector} -- Most types of OpenCV feature detectors (default: {cv2.FastFeatureDetector_create(threshold=25, nonmaxSuppression=True)})
        
        Raises:
            ValueError -- Raised when file either file paths are not correct, or img_file_path is not configured correctly
        '''

        self.lk_params = lk_params
        self.focal = focal_length
        self.pp = pp
        self.R = np.zeros(shape=(3, 3))
        self.t = np.zeros(shape=(3, 3))
        self.id = 0
        self.n_features = 0
        
        self.detector = cv2.FastFeatureDetector_create(threshold=25, nonmaxSuppression=True)

        self.prev_frame = first_image


    def detect(self, img):
        '''Used to detect features and parse into useable format

        
        Arguments:
            img {np.ndarray} -- Image for which to detect keypoints on
        
        Returns:
            np.array -- A sequence of points in (x, y) coordinate format
            denoting location of detected keypoint
        '''

        p0 = self.detector.detect(img)
        
        return np.array([x.pt for x in p0], dtype=np.float32).reshape(-1, 1, 2)


    def visual_odometery(self, frame):
        '''
        Used to perform visual odometery. If features fall out of frame
        such that there are less than 2000 features remaining, a new feature
        detection is triggered. 
        '''
        
        try:
            if self.n_features < 2000:
                self.p0 = self.detect(self.prev_frame)

            # Calculate optical flow between frames, st holds status
            # of points from frame to frame
            self.p1, st, err = cv2.calcOpticalFlowPyrLK(self.prev_frame, frame, self.p0, None, **self.lk_params)

            # Save the good points from the optical flow
            self.good_old = self.p0[st == 1]
            self.good_new = self.p1[st == 1]

            E, _ = cv2.findEssentialMat(self.good_new, self.good_old, self.focal, self.pp, cv2.RANSAC, 0.999, 1.0, None)
            _, R, t, _ = cv2.recoverPose(E, self.good_old, self.good_new, focal=self.focal, pp=self.pp, mask=None)
            # If the frame is one of first two, we need to initalize
            # our t and R vectors so behavior is different
            if self.id < 2:
                self.R = R
                self.t = t
            else:
                # absolute_scale = self.get_absolute_scale()
                absolute_scale = 1
                if (absolute_scale > 0.1 and abs(t[2][0]) > abs(t[0][0]) and abs(t[2][0]) > abs(t[1][0])):
                    self.t = self.t + absolute_scale*self.R.dot(t)
                    self.R = R.dot(self.R)

            # Save the total number of good features
            self.n_features = self.good_new.shape[0]

            # return homogeneous transformation matrix
            T = np.eye(4)
            T[0:3, 0:3] = self.R
            T[0:3, 3] = self.t.reshape(3)

        except Exception:
            T = None

        self.prev_frame = frame
        self.p0 = self.good_new.reshape(-1, 1, 2)
        self.id += 1

        return T