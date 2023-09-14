import numpy as np
from rdp import rdp
import cv2

POINTS_FILE = "points.npy"
EPS = 0.5
LINE_SEGMENTS = 1000
SCALE_FACTOR = 10000
MAP_SIZE = 4000

raw_points = np.load(POINTS_FILE)
pps = len(raw_points) // LINE_SEGMENTS

filtered = []
for i in range(0, LINE_SEGMENTS-1):
    line = rdp(raw_points[i*pps:(i+1)*pps], epsilon=EPS)
    filtered.append(line[0])
    filtered.append(line[1])

filtered = np.array(filtered, dtype=np.float32)
print(filtered.shape)

filtered = filtered*SCALE_FACTOR
filtered[:, 0] += MAP_SIZE//2
filtered[:, 1] = MAP_SIZE//2 - filtered[:, 1]
filtered = filtered.astype(np.int64)


np.save('map.npy', filtered)

# start rgb white image
map_img = np.zeros((MAP_SIZE, MAP_SIZE, 3), dtype=np.uint8)
map_img.fill(255)

# draw coordinate axis x and y
cv2.line(map_img, (0, MAP_SIZE//2), (MAP_SIZE, MAP_SIZE//2), (0, 0, 0), 2)
cv2.line(map_img, (MAP_SIZE//2, 0), (MAP_SIZE//2, MAP_SIZE), (0, 0, 0), 2)
cv2.putText(map_img, "X", (MAP_SIZE - 100, (MAP_SIZE//2) + 100), cv2.FONT_HERSHEY_SIMPLEX, 2, (0, 0, 0), 2)
cv2.putText(map_img, "Y", ((MAP_SIZE//2) - 100, 100), cv2.FONT_HERSHEY_SIMPLEX, 2, (0, 0, 0), 2)

# fix y position to image
print(filtered)
for i in range(0, len(filtered) - 1):
    cv2.line(map_img, filtered[i], filtered[i+1], (0, 0, 0), 2)

for [x, y] in filtered:
    cv2.circle(map_img, (x, y), 3, (0, 255, 0), 2)

# write start and end points
cv2.circle(map_img, (filtered[0][0], filtered[0][1]), 5, (255, 0, 0), 2)
cv2.circle(map_img, (filtered[-1][0], filtered[-1][1]), 5, (0, 0, 255), 2)
cv2.putText(map_img, "start", (filtered[0][0] + 10, filtered[0][1] + 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0), 2)
cv2.putText(map_img, "end", (filtered[-1][0] + 10, filtered[-1][1] - 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)

cv2.imwrite("map_img.png", map_img)