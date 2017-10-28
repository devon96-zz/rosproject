import math
import numpy as np
from decimal import Decimal
import scipy.misc as smp
import timeit
from hashlib import sha1
from time import sleep
import time
import heapq

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


# print new_array.shape[0]


class myCounter():
    def __init__(self):
        self.counter = 0
        self.cells_array = []
        self.neighbors = {}


counter = myCounter()
# print new_array[new_array.shape[0] - 1, new_array.shape[1] - 1]

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


# print find_closest_cell(500, 250)
# print counter.cells_array[find_closest_cell(320, 835)]
# data[520][220] = [0, 220, 222]

for i in range(0, len(counter.cells_array)):
    counter.neighbors[i] = set()

cell_ite = 0
for i in counter.cells_array:
    center_x = i[0]
    center_y = i[1]
    width = i[2]
    height = i[3]

    # center_x = 318
    # center_y = 835
    # width = 13
    # height = 16

    ite = center_x - width / 2 + 1
    while ite <= (center_x + width / 2):  # Check neighbours to the right.
        cell = find_closest_cell(ite, center_y + height / 2 + 2)
        if cell == -1:
            ite += 10
        else:
            # data[counter.cells_array[cell][0]
            #      ][counter.cells_array[cell][1]] = [0, 255, 0]
            counter.neighbors[cell_ite].add(cell)
            counter.neighbors[cell].add(cell_ite)
            ite += counter.cells_array[cell][3]

    ite = center_y - height / 2
    while ite < (center_y + height / 2):  # Check neighbours above.
        cell = find_closest_cell(center_x - width / 2 - 2, ite)
        if cell == -1:
            ite += 10
        else:
            # data[counter.cells_array[cell][0]
            #      ][counter.cells_array[cell][1]] = [0, 255, 0]
            counter.neighbors[cell_ite].add(cell)
            counter.neighbors[cell].add(cell_ite)
            ite += counter.cells_array[cell][2]

    cell_ite += 1


counter.cells_array.append((316, 840, 6, 15))
counter.neighbors[len(counter.cells_array) -
                  1] = set([225, len(counter.cells_array)])
counter.neighbors[225].add(len(counter.cells_array) - 1)

counter.cells_array.append((313, 865, 6, 15))
counter.neighbors[len(counter.cells_array) -
                  1] = set([len(counter.cells_array) - 2, len(counter.cells_array)])

counter.cells_array.append((313, 880, 6, 15))
counter.neighbors[len(counter.cells_array) -
                  1] = set([len(counter.cells_array) - 2, len(counter.cells_array)])

counter.cells_array.append((313, 895, 6, 15))
counter.neighbors[len(counter.cells_array) -
                  1] = set([len(counter.cells_array) - 2, 201])
counter.neighbors[201].add(len(counter.cells_array) - 2)


import pickle

with open('neighbours.pickle', 'wb') as f:
    pickle.dump(counter.neighbors, f)

with open('cells.pickle', 'wb') as f:
    pickle.dump(counter.cells_array, f)

# np.save('neighbours.npy',  counter.neighbors)
# np.save('cells.npy',  counter.cells_array)


# print counter.neighbors[672]


# print graph.get_nodes()


# center_x = counter.cells_array[cell][0]
# center_y = counter.cells_array[cell][1]
# width = counter.cells_array[cell][2]
# height = counter.cells_array[cell][3]

# data[center_x][center_y] = [0, 255, 0]

# cell = find_closest_cell(center_x, center_y + height / 2 + 1)
# data[counter.cells_array[cell][0]][counter.cells_array[cell][1]] = [0, 255, 0]

# cell = find_closest_cell(center_x, center_y - height / 2 - 2)
# data[counter.cells_array[cell][0]][counter.cells_array[cell][1]] = [0, 255, 0]

# cell = find_closest_cell(center_x + width / 2 + 1, center_y)
# data[counter.cells_array[cell][0]][counter.cells_array[cell][1]] = [0, 255, 0]

# cell = find_closest_cell(center_x - width / 2 - 1, center_y)
# data[counter.cells_array[cell][0]][counter.cells_array[cell][1]] = [0, 255, 0]

for i in range(1, new_array.shape[0]):
    for j in range(1, new_array.shape[1]):
        if new_array[i][j] == 1:
            data[i][j] = [0, 0, 255]

# # Hard-coding this pesky, narrow passage since it's too annoying ._.


# print counter.cells_array[len(counter.cells_array) - 1]

# data[310][850] = [255, 0, 0]
# data[313][850] = [0, 255, 0]
# data[316][850] = [255, 0, 0]

# data[310][865] = [255, 0, 0]
# data[313][865] = [0, 255, 0]
# data[316][865] = [255, 0, 0]

# data[310][880] = [255, 0, 0]
# data[313][880] = [0, 255, 0]
# data[316][880] = [255, 0, 0]

# data[310][895] = [255, 0, 0]
# data[313][895] = [0, 255, 0]
# data[316][895] = [255, 0, 0]

# data[counter.cells_array[227][0]][counter.cells_array[227][1]] = [0, 255, 0]

# data[680][140] = [0, 255, 0]
# print find_closest_cell(680, 140)

data[305][910] = [0, 255, 0]
print find_closest_cell(305, 910)

data[315][830] = [0, 255, 0]
print find_closest_cell(315, 830)

# print counter.neighbors[461]
# print counter.neighbors[462]

# data[316][830] = [0, 255, 0]
# print find_closest_cell(316, 830)
# print counter.cells_array[218]

# print find_closest_cell(305, 910)

img = smp.toimage(data)       # Create a PIL image
img.show()  # View in default viewer
