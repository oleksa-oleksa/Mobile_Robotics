#!/usr/bin/env python
# Line detection using HoughLine method
from copy import copy
import sys
import cv2
from matplotlib import pyplot as plt
import numpy as np
from cmath import rect

def detect_line(img, edges, color):
    # copy for OpenCV GBR 
    img_red = np.copy(img)
    found = np.copy(img) 
    
    # This returns an array of r and theta values 
    lines = cv2.HoughLines(edges, 2, np.pi/180, 150) 
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
    img = cv2.cvtColor(img_red, cv2.COLOR_BGR2RGB)  
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
    img = cv2.imread("lines1.jpg", cv2.IMREAD_COLOR)
    img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)

    #=======================================
    # Solution 1
    # Detect line on grayscale image
    # Convert the img to grayscale 
    gray = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY) 
    gray = cv2.GaussianBlur(gray, (5, 5), 0)
    # Apply edge detection method on the image
    threshold1 = 120
    threshold2 = 150
    apertureSize = 2
    edges = cv2.Canny(gray, threshold1, threshold2, apertureSize) 
    # Fill the 2/5 of picture with a black color
    h, w = edges.shape
    cv2.rectangle(edges, (0,0), (w, 2*h/5), 0, cv2.FILLED)
    
    found, lines = detect_line(img, edges, '01grey')
    titles = ('original', 'edges', 'mapped', 'lines')
    images = (gray, edges, found, lines)
    plot_images(titles, images)
    
    #=====================================
    # Solution 2
    # Using bitwise_and 
    # Take each frame
    # Convert BGR to HSV
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    
    #define range of white
    sensitivity = 100
    lower_white = np.array([0, 0, 255 - sensitivity])
    upper_white = np.array([255, sensitivity, 255])
    
    # Threshold the HSV image to get only blue colors
    mask = cv2.inRange(hsv, lower_white, upper_white)
    
    # Bitwise-AND mask and original image
    res = cv2.bitwise_and(img, img, mask= mask)
    # Apply edge detection method on the image
    threshold1 = 50
    threshold2 = 200
    apertureSize = 3
    
    edges = cv2.Canny(res, threshold1, threshold2, apertureSize) 
   
    # Fill the 2/5 of picture with a black color
    h, w = edges.shape
    cv2.rectangle(edges, (0,0), (w, 2*h/5), 0, cv2.FILLED)
   
    found, lines = detect_line(img, edges, '02hsv')
    titles = ('mask', 'bitwise_and', 'edges', 'mapped')
    images = (mask, res, edges, found)
    plot_images(titles, images)
            
    #=====================================
    cv2.waitKey()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)