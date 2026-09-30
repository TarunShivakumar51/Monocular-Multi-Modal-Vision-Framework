import numpy as np
import cv2 as cv

sift = cv.SIFT_create()
bf = cv.BFMatcher()


def compute_keypoints(frame1: np.ndarray, frame2: np.ndarray):
    """
    Returns (points1, points2) between two BGR frames.
    points1/points2 are the matched keypoint coordinates (Nx2).
    """
    gray1 = cv.cvtColor(frame1, cv.COLOR_BGR2GRAY)
    gray2 = cv.cvtColor(frame2, cv.COLOR_BGR2GRAY)

    kp1, des1 = sift.detectAndCompute(gray1, None)
    kp2, des2 = sift.detectAndCompute(gray2, None)
    if des1 is None or des2 is None:
        raise ValueError("No features found in frame1 or frame2")

    matches_ab = bf.knnMatch(des1, des2, k=2)
    matches_ba = bf.knnMatch(des2, des1, k=2)

    good_ab = [p[0] for p in matches_ab if len(p) == 2 and p[0].distance < 0.75 * p[1].distance]
    good_ba = [p[0] for p in matches_ba if len(p) == 2 and p[0].distance < 0.75 * p[1].distance]

    ba_lookup = {m.queryIdx: m.trainIdx for m in good_ba}  # idx in frame2 -> idx in frame1
    mutual = [m for m in good_ab if ba_lookup.get(m.trainIdx) == m.queryIdx]

    points1 = np.float32([kp1[m.queryIdx].pt for m in mutual])
    points2 = np.float32([kp2[m.trainIdx].pt for m in mutual])

    return points1, points2
