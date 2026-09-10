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

    matches = bf.knnMatch(des1, des2, k=2)

    # Apply ratio test
    good = [m for m, n in matches if m.distance < 0.75 * n.distance]

    points1 = np.float32([kp1[m.queryIdx].pt for m in good])
    points2 = np.float32([kp2[m.trainIdx].pt for m in good])

    return points1, points2
