#!/usr/bin/env python
# Line detection using HoughLine method
from copy import copy
import sys
import cv2
from matplotlib import pyplot as plt
import numpy as np


def detect_line(img, edges, color):
    # copy for OpenCV GBR 
    img_red = np.copy(img)
    found = np.copy(img) 
    
    # This returns an array of r and theta values 
    lines = cv2.HoughLines(edges, 1, np.pi/180, 200) 
    # The below for loop runs till r and theta values  
    # are in the range of the 2d array 
    only_lines = np.full(found.shape, (255, 255, 255), dtype=np.uint8)

    for row in lines:
        
        r, theta = row[0]
        # Stores the value of cos(theta) in a 
        a = np.cos(theta) 
      
        # Stores the value of sin(theta) in b 
        b = np.sin(theta) 
          
        # x0 stores the value rcos(theta) 
        x0 = a*r 
          
        # y0 stores the value rsin(theta) 
        y0 = b*r 
          
        # x1 stores the rounded off value of (rcos(theta)-1000sin(theta)) 
        x1 = int(x0 + 1000*(-b)) 
          
        # y1 stores the rounded off value of (rsin(theta)+1000cos(theta)) 
        y1 = int(y0 + 1000*(a)) 
      
        # x2 stores the rounded off value of (rcos(theta)+1000sin(theta)) 
        x2 = int(x0 - 1000*(-b)) 
          
        # y2 stores the rounded off value of (rsin(theta)-1000cos(theta)) 
        y2 = int(y0 - 1000*(a)) 
          
        # create a new blank image
        # cv2.line draws a line in img from the point(x1,y1) to (x2,y2). 
        # (0,0,255) denotes the colour of the line to be  
        #drawn. In this case, it is red for OpenCV function and blue for matplotlib
        cv2.line(found,(x1,y1), (x2,y2), (255,0,0),2)
        cv2.line(img_red,(x1,y1), (x2,y2), (0,0,255),2)
        cv2.line(only_lines,(x1,y1), (x2,y2), (255,0,0),2)
           
    # All the changes made in the input image are finally written on a new image
    file_name = 'linesDetected_' + color + '.jpg'  
    cv2.imwrite(file_name, img_red) 

    return found, only_lines
    
#==========================================    
def plot_images(titles, images):
    rows = 2
    cols = 2
    for i in xrange(len(images)):
        plt.subplot(rows,cols,i+1),plt.imshow(images[i],'gray')
        plt.title(titles[i])
        plt.xticks([]),plt.yticks([])
    
    plt.show()

#========================================== 
def main(args):
    img = cv2.imread("lines.jpg")
    img = cv2.resize(img, (1000, 800))
    
    #=======================================
    # Detect line on grayscale image
    # Convert the img to grayscale 
    gray = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY) 
    gray = cv2.GaussianBlur(gray, (5, 5), 0)
    # Apply edge detection method on the image
    threshold1 = 120
    threshold2 = 350
    apertureSize = 2
    edges = cv2.Canny(gray, threshold1, threshold2, apertureSize) 
    
    found, lines = detect_line(img, edges, '01grey')
    titles = ('orig', 'edges', 'mapped', 'lines')
    images = (gray, edges, found, lines)
    plot_images(titles, images)
    
    #=====================================
    # Detect lines on split RBG image
    b,g,r = cv2.split(img)
    b = cv2.GaussianBlur(b, (5, 5), 0)
    g = cv2.GaussianBlur(g, (5, 5), 0) 
    r = cv2.GaussianBlur(r, (5, 5), 0)
    
    edges = cv2.Canny(b, threshold1, threshold2, apertureSize) 
    detect_line(img, edges, '02blue')

    edges = cv2.Canny(g, threshold1, threshold2, apertureSize) 
    detect_line(img, edges, '03green')
 
    edges = cv2.Canny(r, threshold1, threshold2, apertureSize) 
    detect_line(img, edges, '04red')
    
    #=====================================
    # Detect lines on split LAB image
    l, _, _ = cv2.split(img)
    l = cv2.GaussianBlur(l, (5, 5), 0)
    
    edges = cv2.Canny(l, threshold1, threshold2, apertureSize) 
    detect_line(img, edges, '05lightness')
    
    #=====================================

    

    #=====================================
    cv2.waitKey()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)