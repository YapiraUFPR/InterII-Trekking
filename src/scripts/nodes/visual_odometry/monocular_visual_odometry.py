import cv2
import numpy as np

class MonocularVisualOdometry():

    def __init__(self, first_img) -> None:
        
        # params for ORB feature detection
        self.orb = cv2.ORB_create()
        self.bf = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=True)
        self.last = None

        # params for lucas kanade optical flow
        self.lk_params = dict(winSize=(21, 21),
                              criteria=(cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 30, 0.03))
        
        # params for shi-tomasi corner detection
        self.feature_params = dict( maxCorners = 100,
                              qualityLevel = 0.3,
                              minDistance = 7,
                              blockSize = 7 )
        
        self.prev_img = first_img
        self.p0 = cv2.goodFeaturesToTrack(first_img, mask=None, **self.feature_params)
        self.mask = np.zeros_like(first_img)
        
    def estimate(self, curr_img):

        p1, st, err = cv2.calcOpticalFlowPyrLK(self.prev_img, curr_img, self.p0, None, **self.lk_params)
        # Select good points
        if p1 is not None:
            good_new = p1[st==1]
            good_old = self.p0[st==1]

            print(good_new)
            print(good_old)

            # estimate motion
            for i, (new, old) in enumerate(zip(good_new, good_old)):
                a, b = new.ravel()
                c, d = old.ravel()
                a, b, c, d = int(a), int(b), int(c), int(d)

                self.mask = cv2.line(self.mask, (a, b), (c, d), (0,0,255), 2)
                frame = cv2.circle(curr_img, (a, b), 3, (0,255,0), -1)
                output = cv2.add(frame, self.mask)
                cv2.imshow("sparse optical flow", output)
                cv2.waitKey(1)

            # update
            self.prev_img = curr_img
            self.p0 = good_new.reshape(-1, 1, 2)


        return None, None