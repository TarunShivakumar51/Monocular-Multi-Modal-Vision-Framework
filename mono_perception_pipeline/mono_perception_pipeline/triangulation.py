import cv2 as cv
import numpy as np

def compute_point_cloud(R, t, mtx, points1, points2):
    """
    Triangulates matched points (Nx2) from two views.
    Camera 1 is the origin and camera 2 is at R, t relative to it (X2 = R @ X1 + t).
    Returns an Mx3 array of 3D points in camera 1's frame.
    """
    if len(points1) == 0:
        return np.zeros((0, 3))

    proj_matrix1 = mtx @ np.hstack((np.eye(3), np.zeros((3, 1))))
    proj_matrix2 = mtx @ np.hstack((R, t))

    # triangulatePoints expects 2xN arrays
    points4D = cv.triangulatePoints(proj_matrix1, proj_matrix2, points1.T, points2.T)

    # drop points at infinity before dividing by w
    w = points4D[3]
    valid = np.abs(w) > 1e-8
    points3D = (points4D[:3, valid] / w[valid]).T

    # keep only points in front of both cameras
    z1 = points3D[:, 2]
    z2 = (R @ points3D.T + t)[2]

    point_cloud = points3D[(z1 > 0) & (z2 > 0)]

    return point_cloud
