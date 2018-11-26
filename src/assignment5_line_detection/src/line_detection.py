#!/usr/bin/env python
# Line detection using HoughLine method
from copy import copy
import sys
import cv2
from matplotlib import pyplot as plt
import numpy as np

def region_of_interest(img, vertices):
    mask = np.zeros_like(img)
    channel_count = img.shape[2]
    match_mask_color = (255,) * channel_count
    cv2.fillPoly(mask, vertices, match_mask_color)
    masked_image = cv2.bitwise_and(img, mask)
    return masked_image

def region_of_interest_vertices(img):
    width, height, _ = img.shape
    return [
        (0, height),
        (width / 2, height / 2),
        (width, height),
  ]

def detect_line(img, edges, color):
    # copy for OpenCV GBR 
    img_red = np.copy(img)
    found = np.copy(img) 
    
    # This returns an array of r and theta values 
    lines = cv2.HoughLines(edges, 2, np.pi/180, 200) 
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
def draw_lines(img, lines, color=[255, 0, 0], thickness=3):
    # If there are no lines to draw, exit.
    if lines is None:
        return
    # Make a copy of the original image.
    img = np.copy(img)
    # Create a blank image that matches the original in size.
    line_img = np.zeros(
        (
            img.shape[0],
            img.shape[1],
            3
        ),
        dtype=np.uint8,
    )
    # Loop over all lines and draw them on the blank image.
    for line in lines:
        for x1, y1, x2, y2 in line:
            cv2.line(line_img, (x1, y1), (x2, y2), color, thickness)
    # Merge the image with the lines onto the original.
    img = cv2.addWeighted(img, 0.8, line_image, 1.0, 0.0)
    # Return the modified image.
    return img

#========================================== 
def main(args):
    img = cv2.imread("lines2.png", 1)
    img = cv2.resize(img, (1000, 800))
    
    #=======================================
    # Detect line on grayscale image
    # Convert the img to grayscale 
    gray = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY) 
    gray = cv2.GaussianBlur(gray, (5, 5), 0)
    # Apply edge detection method on the image
    threshold1 = 50
    threshold2 = 250
    apertureSize = 2
    edges = cv2.Canny(gray, threshold1, threshold2, apertureSize) 
    plt.figure()
    plt.imshow(edges)
    plt.show()
    found, lines = detect_line(img, edges, '01grey')
    titles = ('original', 'edges', 'mapped', 'lines')
    images = (gray, edges, found, lines)
    plot_images(titles, images)
    '''
    #=====================================
    # Defining the Region of Interest
    # Shape: a triangle that begins at the bottom left corner of the image, 
    # proceeds to the center of the image at the horizon, and then follows another edge to the bottom right corner of the image. 
    cropped_image = region_of_interest(
    img,
    np.array(
        region_of_interest_vertices(img),
        np.int32
        ),
    )
    gray_image = cv2.cvtColor(cropped_image, cv2.COLOR_RGB2GRAY)
    cannyed_image = cv2.Canny(gray_image, 200, 300)
    
    lines = cv2.HoughLinesP(
    cannyed_image,
    rho=6,
    theta=np.pi / 60,
    threshold=160,
    lines=np.array([]),
    minLineLength=40,
    maxLineGap=25
    )
    
    line_image = draw_lines(img, lines) # <---- Add this call.
    plt.figure()
    plt.imshow(line_image)
    plt.show()
    '''
    #=====================================
    cv2.waitKey()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)