import numpy as np
from rdp import rdp
import cv2

POINTS_FILE = "points.npy"
EPS = 0.5

raw_points = np.load(POINTS_FILE)

filtered = rdp(raw_points, epsilon=EPS)

np.save('map.npy', raw_points)

raw_img = map_img = np.ones((100, 100), dtype=np.uint8)
for [x, y] in raw_points:
    cv2.circle(map_img, (x, y), 3, (255, 255, 255), 2)
cv2.imwrite("raw_img.png", raw_img)

map_img = np.ones((100, 100), dtype=np.uint8)
for [x, y] in filtered:
    cv2.circle(map_img, (x, y), 3, (255, 255, 255), 2)

for i in range(0, len(filtered) - 1):
    cv2.line(map_img, filtered[i], filtered[i+1], (255, 255, 255), 2)

cv2.imwrite("map_img.png", map_img)