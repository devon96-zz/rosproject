import math
import numpy as np
from decimal import Decimal
import scipy.misc as smp
import timeit
from hashlib import sha1
from time import sleep
import time
import heapq
import pickle

# Create a 1024x1024x3 array of 8 bit unsigned integers
f = open("map.txt", "r")
array = f.read()
array = array[1:-1]

new_array = np.fromstring(array, sep=', ', dtype=np.int16)
new_array = np.where(new_array == 100, 1, new_array)
new_array = np.reshape(new_array, (-1, 1000))
new_array = np.flipud(new_array)

new_array[:, 0] = 0
new_array[0, :] = 0


data = np.zeros((800, 1000, 3), dtype=np.uint8)

x_size = new_array.shape[0]
y_size = new_array.shape[1]


class myCounter():
    def __init__(self):
        self.counter = 0
        self.cells_array = []
        self.neighbors = {}


counter = myCounter()

for i in range(6, new_array.shape[0] - 6):
    for j in range(6, new_array.shape[1] - 6):
        if new_array[i][j] == 1:
            for k in range(1, 3):
                new_array[i + k][j + k] = 100
                new_array[i - k][j + k] = 100
                new_array[i + k][j - k] = 100
                new_array[i - k][j - k] = 100

                new_array[i][j + k] = 100
                new_array[i][j - k] = 100
                new_array[i + k][j] = 100
                new_array[i - k][j] = 100
new_array = np.where(new_array == 100, 1, new_array)


def draw_lines(arr, depth):
    if 1 not in arr:
        arr[0, :] = 1
        arr[:, 0] = 1
        arr[arr.shape[0] / 2, arr.shape[1] / 2] = arr.shape[0]
    elif depth == 0:
        return
    else:
        draw_lines(arr[:arr.shape[0] / 2, :arr.shape[1] / 2],
                   depth - 1)  # Top left

        draw_lines(arr[:arr.shape[0] / 2, arr.shape[1] / 2:],
                   depth - 1)  # Top right

        draw_lines(arr[arr.shape[0] / 2:, :arr.shape[1] / 2],
                   depth - 1)  # Bottom left

        draw_lines(arr[arr.shape[0] / 2:, arr.shape[1] / 2:],
                   depth - 1)  # Bottom right


draw_lines(new_array, 6)

for i in range(0, new_array.shape[0]):
    for j in range(0, new_array.shape[1]):
        if new_array[i][j] > 1:
            counter.cells_array.append((i, j, new_array[i][j], int(
                round((new_array[i][j] * new_array.shape[1]) / new_array.shape[0]))))

tmp = counter.cells_array[225]
counter.cells_array[225] = (tmp[0] - 5, tmp[1], tmp[2], tmp[3])

tmp = counter.cells_array[224]
counter.cells_array[224] = (tmp[0] - 3, tmp[1], tmp[2], tmp[3])

tmp = counter.cells_array[201]
counter.cells_array[201] = (tmp[0] + 5, tmp[1], tmp[2], tmp[3])

for i in counter.cells_array:
    data[i[0]][i[1]] = [255, 0, 0]


def find_closest_cell(x, y):
    min_dist = 9999
    cell_index = -1
    ite = 0
    for i in counter.cells_array:
        dist = math.sqrt(i[0]**2 + i[1]**2)
        if((dist < min_dist) and
           (x in range(i[0] - i[2] / 2, i[0] + i[2] / 2) and
                (y in range(i[1] - i[3] / 2, i[1] + i[3] / 2)))):
            cell_index = ite
            min_dist = dist
        ite += 1
    return cell_index


for i in range(0, len(counter.cells_array)):
    counter.neighbors[i] = set()

cell_ite = 0
for i in counter.cells_array:
    center_x = i[0]
    center_y = i[1]
    width = i[2]
    height = i[3]

    ite = center_x - width / 2 + 1
    while ite <= (center_x + width / 2):  # Check neighbours to the right.
        cell = find_closest_cell(ite, center_y + height / 2 + 2)
        if cell == -1:
            ite += 10
        else:
            counter.neighbors[cell_ite].add(cell)
            counter.neighbors[cell].add(cell_ite)
            ite += counter.cells_array[cell][3]

    ite = center_y - height / 2
    while ite < (center_y + height / 2):  # Check neighbours above.
        cell = find_closest_cell(center_x - width / 2 - 2, ite)
        if cell == -1:
            ite += 10
        else:
            counter.neighbors[cell_ite].add(cell)
            counter.neighbors[cell].add(cell_ite)
            ite += counter.cells_array[cell][2]

    cell_ite += 1

# Hard-coding this pesky, narrow passage since it's too annoying ._.
counter.cells_array.append((313, 840, 6, 15))
counter.neighbors[len(counter.cells_array) -
                  1] = set([225, len(counter.cells_array)])
counter.neighbors[225].add(len(counter.cells_array) - 1)

counter.cells_array.append((313, 865, 6, 15))
counter.neighbors[len(counter.cells_array) -
                  1] = set([len(counter.cells_array) - 2, len(counter.cells_array)])

counter.cells_array.append((313, 880, 6, 15))
counter.neighbors[len(counter.cells_array) -
                  1] = set([len(counter.cells_array) - 2, len(counter.cells_array)])

counter.cells_array.append((312, 895, 6, 15))
counter.neighbors[len(counter.cells_array) -
                  1] = set([len(counter.cells_array) - 2, 201])
counter.neighbors[201].add(len(counter.cells_array) - 2)


with open('neighbours.pickle', 'wb') as f:
    pickle.dump(counter.neighbors, f)

with open('cells.pickle', 'wb') as f:
    pickle.dump(counter.cells_array, f)
