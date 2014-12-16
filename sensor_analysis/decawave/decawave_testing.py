# Eric Schneider

import json
import numpy as np
import matplotlib.pyplot as plt

if __name__ == '__main__':
    with open("data_12_15_14.txt") as json_file:
        data = json.load(json_file)

    measured = np.array(0)
    actual = np.array(0)
    orientation = []

    for point in data:
        print point
        measured = np.append(measured, point[0])
        actual = np.append(actual, point[1])
        orientation.append(str(point[2]))

    # The data was confusing viewed all at once, so this makes two views
    # axes = ([-1, 10, -1, 10], [10, 91, 10, 100])  # 12-11-14
    axes = ([90, 110, 90, 110], [95, 105, 95, 105])  # 12-15-14
    for axis in axes:
        plt.plot([0, 45], [0, 44], 'r', linewidth=2, zorder=1)
        plt.plot([0, 45], [0, 46], 'r', linewidth=2, zorder=1)
        plt.plot([45, 110], [44, 109], 'r', linewidth=2, zorder=1)
        plt.plot([45, 110], [46, 111], 'r', linewidth=2, zorder=1)
        plt.plot([0, 110], [0, 110], 'g', linewidth=2, zorder=1)
        plt.scatter(actual, measured, 40, zorder=2)
        plt.xlabel("Actual distance (m)")
        plt.ylabel("Measured distance (m)")
        plt.title("Actual vs. Measured for decawave outside. Red lines are approximations of measurement accuracy")

        plt.axis(axis)
        plt.show()
