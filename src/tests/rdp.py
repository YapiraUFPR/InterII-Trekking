import numpy as np
from rdp import rdp
import matplotlib.pyplot as plt

raw_points = np.load("data.npy")
filtered_points = rdp(raw_points, epsilon=0.5)

x, y = zip(*filtered_points)