#!/usr/bin/env python
# Line detection using HoughLine method
import sys
import cv2
from matplotlib import pyplot as plt
import numpy as np
from sklearn import linear_model

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
    
def line_segments(img, limit=2):
    """
    Get two lines by finding the outlines of the mask
    :param img:
    :return: a list with two outlines sorted by the largest to the smallest
    """

    (_, contours, _) = cv2.findContours(img.copy(), cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    contours = sorted(contours, key=cv2.contourArea, reverse=True)

    return contours[:limit]


def ransac_method(contour):
    """
    Get the linear model of lines
    :param contour: line contour
    :return: parameters m and b of the linear model
    """

    ransac = linear_model.RANSACRegressor()

    x = []
    y = []
    for val in contour:
        x.append([val[0][1]])
        y.append([val[0][0]])

    ransac.fit(x, y)
    b = ransac.estimator_.intercept_
    m = ransac.estimator_.coef_
    return m[0][0], b[0]

def end_start_points(m, b, width):
    """
    Gets two pair of coordinates
    :param m:
    :param b:
    :param width:
    :return:
    """
    x1 = 0
    x2 = width
    y1 = (x1 * m + b)
    y2 = (x2 * m + b)
    return ((y1,x1),(y2,x2))


def show_lines(img, lines):
    
    img = img.copy()
    
    for line in lines:
        l0 = tuple(int(c) for c in line[0])
        l1 = tuple(int(c) for c in line[1])
        cv2.line(img, l0, l1, (255, 0, 0), 5)

    return img

#========================================== 
def main(args):
    img = cv2.imread("lines1.jpg", 1)
    #img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)

    #=======================================
    # Solution 1
    # RANSAC
    
    # Crop 20% of the image along the y axis
    y_end = np.shape(img)[0]
    y_start = (np.shape(img)[0] * 0.2)
    img = img[int(y_start): int(y_end), :]

    # Convert RGB to HSV
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

    # define range of color in HSV
    lower = np.array([0, 40, 150])
    upper = np.array([18, 80, 255])

    # Threshold the HSV image to get only the lines colors
    mask = cv2.inRange(hsv, lower, upper)

    # Bitwise-AND mask and original image
    res = cv2.bitwise_and(img, img, mask=mask)

    seg1, seg2 = line_segments(mask)

    m1, b1 = ransac_method(seg1)
    print("Equation line 1: y1 = %fx + %f" % (m1, b1))
    m2, b2 = ransac_method(seg2)
    print("Equation line 2: y2 = %fx + %f" % (m2, b2))

    line1 = end_start_points(m1, b1, img.shape[1])
    line2 = end_start_points(m2, b2, img.shape[1])

    ransac_lines = show_lines(img, line1, line2)
    titles = ('original', 'mask', 'res', 'ransac lines')
    images = (img, mask, res, ransac_lines)
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
   
    found, _ = detect_line(img, edges, '02hsv')
    titles = ('mask', 'bitwise_and', 'edges', 'hough lines')
    images = (mask, res, edges, found)
    plot_images(titles, images)

    #====================================
    # Solution 3
    # Using Hough Method for line detection 
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
    titles = ('original', 'edges', 'hough lines', 'lines')
    images = (gray, edges, found, lines)
    plot_images(titles, images)
            
    #=====================================
    cv2.waitKey()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)