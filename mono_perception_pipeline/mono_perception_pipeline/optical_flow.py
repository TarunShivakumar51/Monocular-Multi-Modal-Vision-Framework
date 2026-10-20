import numpy as np
import cv2 as cv

feature_params = dict(maxCorners=100,
                      qualityLevel=0.3,
                      minDistance=7,
                      blockSize=7)

lk_params = dict(winSize=(15, 15),
                 maxLevel=2,
                 criteria=(cv.TERM_CRITERIA_EPS | cv.TERM_CRITERIA_COUNT, 10, 0.03))


def compute_optical_flow(frame1: np.ndarray, frame2: np.ndarray, mtx: np.ndarray = None):
    """
    Returns (R, t, points1, points2) between two BGR frames.
    R, t map points from frame1's camera to frame2's camera: X2 = R @ X1 + t.
    points1/points2 are the inlier keypoint coordinates (Nx2) that agree with R, t.
    mtx is the 3x3 camera intrinsics matrix; None falls back to focal=1, pp=(0, 0).
    """
    gray1 = cv.cvtColor(frame1, cv.COLOR_BGR2GRAY)
    gray2 = cv.cvtColor(frame2, cv.COLOR_BGR2GRAY)

    p0 = cv.goodFeaturesToTrack(gray1, mask=None, **feature_params)
    if p0 is None:
        raise ValueError("No features found in frame1")

    p1, st, _ = cv.calcOpticalFlowPyrLK(gray1, gray2, p0, None, **lk_params)
    
    if p1 is not None:
        good_old = p0[st == 1]
        good_new = p1[st == 1]
    else:
         return None    

    if len(good_old) < 20 or len(good_new) < 20:
        return None

    if mtx is None:
        E, mask = cv.findEssentialMat(good_old, good_new, focal=1.0, pp=(0.0, 0.0),
                                      method=cv.RANSAC, prob=0.999, threshold=1.0)
        if E.shape != (3,3):
            return None

        _, R, t, mask = cv.recoverPose(E, good_old, good_new, focal=1.0, pp=(0.0, 0.0), mask=mask)

    else:
        E, mask = cv.findEssentialMat(good_old, good_new, mtx,
                                      method=cv.RANSAC, prob=0.999, threshold=1.0)

        if E.shape != (3,3):
                    return None
        
        _, R, t, mask = cv.recoverPose(E, good_old, good_new, mtx, mask=mask)

    # keep only points that passed RANSAC and are in front of both cameras
    inliers = mask.ravel() > 0
    inliers_length = np.sum(inliers)
    if inliers_length < 15:
         return None

    return R, t, good_old[inliers], good_new[inliers]
