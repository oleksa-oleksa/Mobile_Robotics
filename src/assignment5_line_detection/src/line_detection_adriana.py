#!/usr/bin/env python
import cv2
import numpy as np
from sklearn import linear_model


def line_segments(img):
    """
    Get two lines by finding the outlines of the mask
    :param img:
    :return: a list with two outlines sorted by the largest to the smallest
    """

    (_, contours, _) = cv2.findContours(img.copy(), cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    contours = sorted(contours, key=cv2.contourArea, reverse=True)[:2]

    return contours[0], contours[1]


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

    return m, b


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


def show_lines(img, line1, line2):
    """
    Draws the calculated lines on the original image
    :param img: original image
    :param line1:
    :param line2:
    :return: two lines drawn
    """
    img = img.copy()
    cv2.line(img, line1[0], line1[1], (255, 0, 0), 5)
    cv2.line(img, line2[0], line2[1], (255, 0, 0), 5)
    return img


if __name__ == '__main__':

    img = cv2.imread('car_picture.jpeg', 1)

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

    img = show_lines(img, line1, line2)
    cv2.imshow('img', img)
    cv2.imshow('mask', mask)
    cv2.imshow('res', res)
    cv2.waitKey(0)
    cv2.destroyAllWindows()
