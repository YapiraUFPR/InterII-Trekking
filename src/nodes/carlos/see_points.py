import numpy as np

raw_data = np.load("points.npy")

for [x, y] in raw_data:
    print(x, y)