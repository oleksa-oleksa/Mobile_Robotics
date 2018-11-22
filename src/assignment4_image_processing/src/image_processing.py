#!/usr/bin/env python
import sys
import cv2
import math
from matplotlib import pyplot as plt
import numpy as np
from matplotlib.mlab import center_matrix
      

def sort_pts(pts):
    sorted_by_y = sorted(pts, key=lambda r: r[1])
    twos = zip(*(iter(sorted_by_y),) * 2)
    sorted_by_x = map(lambda t: sorted(t, key=lambda p: p[0]), twos)
    return np.array([z for l in sorted_by_x for z in l])  

# Checks if a matrix is a valid rotation matrix.
def isRotationMatrix(R) :
    Rt = np.transpose(R)
    shouldBeIdentity = np.dot(Rt, R)
    I = np.identity(3, dtype = R.dtype)
    n = np.linalg.norm(I - shouldBeIdentity)
    return n < 1e-6
 
# Calculates rotation matrix to euler angles
# The result is the same as MATLAB except the order
# of the euler angles ( x and z are swapped ).
def rotationMatrixToEulerAngles(R) :
 
    assert(isRotationMatrix(R))
     
    sy = math.sqrt(R[0,0] * R[0,0] +  R[1,0] * R[1,0])
     
    singular = sy < 1e-6
 
    if  not singular :
        x = math.atan2(R[2,1] , R[2,2])
        y = math.atan2(-R[2,0], sy)
        z = math.atan2(R[1,0], R[0,0])
    else :
        x = math.atan2(-R[1,2], R[1,1])
        y = math.atan2(-R[2,0], sy)
        z = 0
 
    return np.array([x, y, z])

def main(args):
    cv_image = cv2.imread("points_lab_measured.jpeg")
    #=====================================
    #make it gray
    #Task 2: obtain the RGB image from the camera and transform it to gray
    gray=cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
    
    #gauss
    #if the camera image is too noisy it could be made softer with gauss filter
    #dst=cv2.GaussianBlur(cv_image,(5,5),0,0)
    
    #=====================================
    #bi_gray
    #Task 3: turn  your  gray  image  from  the  previous  step  to  a black/white  image.
    bi_gray_max = 255
    bi_gray_min = 245
    ret,thresh1=cv2.threshold(gray, bi_gray_min, bi_gray_max, cv2.THRESH_BINARY);

    titles = ['Original Image', 'GRAY','BINARY']
    images = [cv_image, gray, thresh1]

    cv2.imshow('threshold',thresh1)
    cv2.waitKey()
    print("Done")
    
    #=====================================
    #Task 4: Find the white points in the image
    white_points = []
    height, width = thresh1.shape
    for j in range(height):
        for i in range(width):
            if thresh1[j, i] == 255:
                white_points.append((i, j))
    
    print("Total found:", len(white_points))
    print("Coordinates: ", white_points)

    #=====================================
    #Task 5: Compute the extrinsic parameters
    #Define a 3x3 cv::Mat matrix for the intrinsic parameters and use the following numbers:
    fx = 614.1699
    fy = 614.9002
    cx = 329.9491
    cy = 237.2788
    
    #Define a 4x1 cv::Mat vector for the distortion parameters and use the following numbers:
    k1 = 0.1115
    k2 = - 0.1089
    p1 = 0
    p2 = 0
    #Matrixes
    camera_mat = np.zeros((3,3,1))
    camera_mat[:,:,0] = np.array([[fx, 0, cx],
                                  [0, fy, cy],
                                  [0, 0, 1]])
    dist_coeffs = np.zeros((4,1))
    dist_coeffs[:,0] = np.array([[k1, k2, p1, p2]])
    # far to close, left to right (order of discovery) in cm
    obj_points = np.zeros((6,3,1))

    obj_points[:,:,0] = np.array([[00.0, 00.0, 0],
                                  [40.0, 00.0, 0],
                                  [00.0, 28.0, 0],
                                  [40.0, 27.0, 0],
                                  [00.0, 56.0, 0],
                                  [40.0, 55.5, 0]])
    #Estimate the initial camera pose as if the intrinsic parameters have been already known. 
    #This is done using solvePnP().
    white_points_array = np.array(white_points)
    # convert to np.float32
    white_points_float = np.float32(white_points_array)
    # define criteria and apply kmeans()
    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 10, 1.0)
    K = 6
    ret,label,center = cv2.kmeans(white_points_float, K, None, criteria,10,
                                  cv2.KMEANS_PP_CENTERS)
   
    center = sort_pts(center.tolist())
    obj_points = sort_pts(obj_points.tolist())
    
    for c in center:
        cv2.circle(cv_image, tuple(c.astype(np.int)), 
                   4, (255, 0, 0), 2)
        
    cv2.imshow('threshold',cv_image)
        
    _, rvec, tvec = cv2.solvePnP(obj_points, center, camera_mat, dist_coeffs)
    
    #=====================================
    #Task 5: Finding the camera location and orientation
    rmat = np.zeros((3,3))
    cv2.Rodrigues(rvec, rmat, jacobian=0)
    
    hmat = np.identity(4, np.float32)
    print(hmat)
    hmat[0:3, 0:3] = rmat
    hmat[0:3, 3] = tvec.T
    print("Homogeneous transform:")
    print(hmat)
    
    hmat_inv = np.linalg.inv(hmat)
    print("Inverse Homogeneous transform:")
    print(hmat_inv)
    
    print("Rotation Matrix To Euler Angles:")
    angles = rotationMatrixToEulerAngles(rmat)
    print("x: ", angles[0], "y: ", angles[1], "z: ", angles[2])
    
    print("The world coordinate system's origin in camera's coordinate system:")
    print("=== camera rvec:")
    print(rvec)
    print("=== camera rmat:")
    print(rmat)
    print("=== camera tvec:")
    print(tvec)
    
    print("The camera origin in world coordinate system:")
    print("=== camera rmat:")
    print(rmat.T)
    print("=== camera tvec:")
    print(-np.dot(rmat.T, tvec))
    
    #=====================================
    cv2.waitKey()
    cv2.destroyAllWindows()

if __name__ == '__main__':
   main(sys.argv)
   