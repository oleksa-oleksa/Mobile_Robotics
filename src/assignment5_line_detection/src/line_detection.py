#!/usr/bin/env python
# A line can be represented as y = mx + c 
# or in parametric form, as r = xcosθ + ysinθ where r is the perpendicular distance 
# from origin to the line, and θ is the angle formed by this perpendicular line 
# and horizontal axis measured in counter-clockwise

# Line detection using HoughLine method 
import sys
import cv2
import math
from matplotlib import pyplot as plt
import numpy as np
from matplotlib.mlab import center_matrix

def main(args):
    cv_image = cv2.imread("lines.jpg")

    
    #=====================================
    cv2.waitKey()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)