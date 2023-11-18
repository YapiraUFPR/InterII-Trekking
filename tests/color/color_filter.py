import cv2
from sys import argv
import numpy as np

IMAGE_PATH = argv[1]
LOWER_BOUND = np.array([20, 0, 0])
UPPER_BOUND = np.array([45, 255, 255])

image = cv2.imread(IMAGE_PATH)

hsv_image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
mask = cv2.inRange(hsv_image, LOWER_BOUND, UPPER_BOUND)
contours, hierarchy = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

bb_image = hsv_image.copy()
if len(contours) > 0:
    
    largest_contour = contours[0]
    bb = cv2.boundingRect(largest_contour)
    largest_area = bb[2] * bb[3]
    for contour in contours:
        bb = cv2.boundingRect(contour)
        area = bb[2] * bb[3]
        if area > largest_area:
            largest_contour = contour
            largest_area = area

    bb = cv2.boundingRect(largest_contour)

    x, y, w, h = bb
    cx, cy = x + w//2, y + h//2
    height, width, _ = hsv_image.shape

# draw bounding box
cv2.rectangle(bb_image, (x, y), (x+w, y+h), (0, 255, 0), 2)      
cv2.circle(bb_image, (cx, cy), 5, (0, 0, 255), -1)
show_img = np.hstack((cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR), cv2.cvtColor(bb_image, cv2.COLOR_HSV2BGR)))
cv2.imshow("out", show_img)
cv2.waitKey(0)