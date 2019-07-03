#!/usr/bin/env python
import cv2
import numpy as np
import matplotlib.pyplot as plt
import sys


def get_point_circle(radius, x, h, k):
    #(h,k) is the circle center
    return round(np.sqrt(radius**2 - (x - h)**2) + k)


def get_contours(img):
    ret, thresh = cv2.threshold(img, 127, 255, cv2.THRESH_BINARY)
    _, contours, _ = cv2.findContours(thresh, 1, 2)
    return contours[26]


def distance_to_the_circle(x, y, h, k, r):
    # (h,k) is the center point of the circle and r is radius
    return abs(np.sqrt((x - h)**2 + (y - k)**2) - r)


def get_intercept_point(x, y, h, k, r): #Closest point to the se circle
    # https://math.stackexchange.com/questions/127613/closest-point-on-circle-edge-from-point-outside-inside-the-circle
    vector_x = x - h
    vector_y = y - k
    magnitud_vector = np.sqrt(vector_x**2 + vector_y**2)
    point_x = round(h + (vector_x / magnitud_vector) * r)
    point_y = round(k + (vector_y / magnitud_vector) * r)
    return point_x, point_y

def find_main_parts(cnt):
    # Split contours into 2 half circles and two straight lines
    # coord[1] --> right line
    # coord[2] --> lower semicircle
    # coord[3] --> left line
    # coord[4] --> upper semicircle
    size_cnt = len(cnt)
    groups = 0
    coord = []
    coord.append([])
    for i in range(1, size_cnt):
        if abs(cnt[i][0][1] - cnt[i - 1][0][1]) > 100:
            groups += 1
            coord.append([])
            coord[groups].append(cnt[i - 1][0])
            coord[groups].append(cnt[i][0])
            groups += 1
            coord.append([])
        coord[groups].append(cnt[i][0])

    for i in range(len(coord)):
        coord[i] = np.array(coord[i])

    coord[4] = np.append(coord[4], coord[0], axis=0)
    return coord


def calculate_shape():
    # Load the image and convert it to gray
    img = cv2.imread('/home/adripinto/catkin_ws_user/src/assignment9/src/map.jpeg')

    gray = cv2.cvtColor(img, cv2.COLOR_RGB2GRAY)

    # Find contours
    cnt = get_contours(gray)

    # Draw contours
    black = np.zeros(gray.shape)
    cv2.drawContours(black, [cnt], -1, (255, 255, 30), 3)
    plt.imshow(black)

    coord = find_main_parts(cnt)

    # Fit ellipses and print center points and radius of the semicircles
    ellipse1 = cv2.fitEllipse(coord[4])
    x1 = ellipse1[0][0]
    y1 = ellipse1[0][1]
    r1 = round(ellipse1[1][0] / 2)
    print("Ellipse1 Center:", (int(round(x1)), int(round(y1))), "Radius:", int(round(r1)))

    ellipse2 = cv2.fitEllipse(coord[2])
    x2 = ellipse2[0][0]
    y2 = ellipse2[0][1]
    r2 = round(ellipse2[1][0] / 2)
    print("Ellipse2 Center:", (int(round(x2)), int(round(y2))), "Radius:", int(round(r2)))

    # Print semicircle equations
    print("Upper semi-circle equation: sqrt(", r1, "**2 - (x -", round(x1), ")**2 + ", round(y1))
    print("Lower semi-circle equation: sqrt(", r2, "**2 - (x -", round(x2), ")**2 + ", round(y2))

    # Print line equations
    print("Equation right-line:", 0, "x +", coord[1][0][0])
    print("Equation left-line:", 0, "x +", coord[3][0][0])

    return img.shape[1], x1, y1, r1, x2, y2, r2, coord[1][0][0], coord[3][0][0]


def get_closest_point(y, x, width, x1, y1, r1, x2, y2, r2, x3, x4):
    x = int(round(float(x) / 0.01))
    y = int(round(float(y) / 0.01))
    print("Coordenates in meters: ", (x, y))
    coord_x = None
    coord_y = None
    distance = None
    if y2 > y > y1:
        if x >= width / 2:
            coord_x = x3
            coord_y = y
            distance = np.abs(x - coord_x) * 0.01
            plt.scatter([coord_x], [coord_y], c='g', s=40)
            print(
                "The given point is closer to the right line at coord = ",
                (int(coord_x), int(coord_y)),
                "Distance in meters:",
                distance
            )
        else:
            coord_x = x4
            coord_y = y
            distance = np.abs(x - coord_x) * 0.01
            plt.scatter([coord_x], [coord_y], c='g', s=40)
            print("The given point is closer to the left line at coord = ",
                  (int(coord_x), int(coord_y)),
                  "Distance in meters:",
                  distance
            )
    elif y <= y1:
        distance = distance_to_the_circle(x, y, x1, y1, r1) * 0.01
        coord_x, coord_y = get_intercept_point(x, y, x1, y1, r1)
        plt.scatter([coord_x], [coord_y], c='g', s=40)
        print(
            "The given point is closer to the upper circle at:",
            (int(coord_x), int(coord_y)),
            ", Distance in meters:",
            distance
        )
    elif y >= y2:
        distance = distance_to_the_circle(x, y, x2, y2, r2) * 0.01
        coord_x, coord_y = get_intercept_point(x, y, x2, y2, r2)
        plt.scatter([coord_x], [coord_y], c='g', s=40)
        print(
            "The given point is closer to the lower circle at:",
            (int(coord_x), int(coord_y)),
            ", Distance in meters:",
            distance
        )

    return (coord_x * 0.01, coord_y * 0.01), distance


def main(argv):
    width, x1, y1, r1, x2, y2, r2, x3, x4 = calculate_shape()
    coords, distance = get_closest_point(argv[0], argv[1], width, x1, y1, r1, x2, y2, r2, x3, x4)

if __name__== "__main__":
  main(sys.argv[1:])
