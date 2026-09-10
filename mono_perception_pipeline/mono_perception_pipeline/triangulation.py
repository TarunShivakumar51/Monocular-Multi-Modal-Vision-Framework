import cv2 as cv
import numpy as np

def compute_point_cloud(R1, t1, R2, t2, mtx, points1, points2):

    R1t1 = np.hstack((R1, t1))
    proj_matrix1 = np.dot(mtx, R1t1)

    R2t2 = np.hstack((R2, t2)) 
    proj_matrix2 = np.dot(mtx, R2t2)

    points4D = cv.triangulatePoints(proj_matrix1, proj_matrix2, points1, points2)

    points3D = points4D[:3, :] / points4D[3, :]

    point_cloud = points3D.T

    return point_cloud


