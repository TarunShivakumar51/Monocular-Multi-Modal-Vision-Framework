import numpy as np
import cv2 as cv

feature_params = dict(maxCorners=100,
                      qualityLevel=0.3,
                      minDistance=7,
                      blockSize=7)

lk_params = dict(winSize=(15, 15),
                 maxLevel=2,
                 criteria=(cv.TERM_CRITERIA_EPS | cv.TERM_CRITERIA_COUNT, 10, 0.03))


def compute_optical_flow(frame1: np.ndarray, frame2: np.ndarray):
    """
    Returns (R, t, points1, points2) between two BGR frames.
    points1/points2 are the matched keypoint coordinates (Nx2) for COLMAP.
    """
    gray1 = cv.cvtColor(frame1, cv.COLOR_BGR2GRAY)
    gray2 = cv.cvtColor(frame2, cv.COLOR_BGR2GRAY)

    p0 = cv.goodFeaturesToTrack(gray1, mask=None, **feature_params)
    if p0 is None:
        raise ValueError("No features found in frame1")

    p1, st, _ = cv.calcOpticalFlowPyrLK(gray1, gray2, p0, None, **lk_params)

    good_old = p0[st == 1]
    good_new = p1[st == 1]

    E, _ = cv.findEssentialMat(good_old, good_new, None,
                               method=cv.RANSAC, prob=0.999, threshold=1.0)
    _, R, t, _ = cv.recoverPose(E, good_old, good_new, None)

    return R, t, good_old, good_new
