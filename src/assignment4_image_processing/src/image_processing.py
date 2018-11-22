#!/usr/bin/env python
import sys
import cv2
from matplotlib import pyplot as plt
import numpy as np
      
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
    
    print("Done")
    

    #=====================================
    #Task 4: Find the white points in the image
    white_points = []
    height, width = thresh1.shape
    for i in range(height):
        for j in range(width):
            if thresh1[i, j] == 255:
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
                                  [21.8, 00.0, 0],
                                  [00.0, 30.0, 0],
                                  [22.2, 30.0, 0],
                                  [00.0, 60.0, 0],
                                  [22.0, 60.0, 0]])
    #Estimate the initial camera pose as if the intrinsic parameters have been already known. 
    #This is done using solvePnP().
    retval, rvec, tvec = cv2.solvePnP(obj_points, white_points, camera_mat, dist_coeffs)
    '''
    rmat = np.zeros((3,3))
    cv2.Rodrigues(rvec, rmat, jacobian=0)
    '''    
    cv2.waitKey()
    cv2.destroyAllWindows()

if __name__ == '__main__':
   main(sys.argv)
   