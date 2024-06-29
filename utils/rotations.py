import numpy as np

def rot_x(theta):
    return np.array([[1, 0, 0],
                     [0, np.cos(theta), -np.sin(theta)],
                     [0, np.sin(theta), np.cos(theta)]])

def rot_y(theta):
    return np.array([[np.cos(theta), 0, np.sin(theta)],
                     [0, 1, 0],
                     [-np.sin(theta), 0, np.cos(theta)]])

def rot_z(theta):
    return np.array([[np.cos(theta), -np.sin(theta), 0],
                     [np.sin(theta), np.cos(theta), 0],
                     [0, 0, 1]])

if __name__ == "__main__":
    v = np.array([-0.34375, -0.53515625, 9.69140625])

    print(np.round(rot_x(np.pi) @ rot_y(np.deg2rad(2.5)) @ rot_x(np.deg2rad(-3)) @ v, 5))

    print(np.round(rot_x(np.pi) @ rot_y(np.deg2rad(2.5)) @ rot_x(np.deg2rad(-3)), 6))
