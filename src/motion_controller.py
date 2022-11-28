#must install bitarray to run
from bitarray import bitarray
from timeit import default_timer as timer
import random

def printBA(keys, BAs):
    for index in range(ROWS*COLUMNS):
        if index % COLUMNS == 0:
            print()
        printed = False
        for value, key in enumerate(keys):
            if BAs[value][index] == 1 and not printed:
                print(key, end=' ')
                printed = True
        if not printed:
            print('.', end=' ')
    print('\n')


def nearestNeighbour1(BA_unknown, BA_wall, BA_path, BA_visited):
    BA_current_search = ((~BA_wall & ((BA_visited & ~BA_down) >> COLUMNS)) |
                         (~BA_wall & ((BA_visited & ~BA_up) << COLUMNS)) |
                         (~BA_wall & ((BA_visited & ~BA_right) >> 1)) |
                         (~BA_wall & ((BA_visited & ~BA_left) << 1)) |
                         (~BA_wall & ((BA_visited & ~BA_down & ~BA_right) >> COLUMNS + 1)) |
                         (~BA_wall & ((BA_visited & ~BA_down & ~BA_left) >> COLUMNS - 1)) |
                         (~BA_wall & ((BA_visited & ~BA_up & ~BA_right) << COLUMNS + 1)) |
                         (~BA_wall & ((BA_visited & ~BA_up & ~BA_left) << COLUMNS - 1))) & ~BA_visited

    if (BA_current_search & BA_unknown).any():
        return BA_path
    else:
        return nearestNeighbour1(BA_unknown, BA_wall, BA_path+[BA_current_search], BA_visited | BA_current_search)


def nearestNeighbour2(BA_path, final_path):
    if len(BA_path) == 1:
        return final_path

    BA_neighbours = (((final_path[-1] & ~BA_down) >> COLUMNS) |
                  ((final_path[-1] & ~BA_up) << COLUMNS) |
                  ((final_path[-1] & ~BA_right) >> 1) |
                  ((final_path[-1] & ~BA_left) << 1) |
                  ((final_path[-1] & ~BA_down & ~BA_right) >> COLUMNS + 1) |
                  ((final_path[-1] & ~BA_down & ~BA_left) >> COLUMNS - 1) |
                  ((final_path[-1] & ~BA_up & ~BA_right) << COLUMNS + 1) |
                  ((final_path[-1] & ~BA_up & ~BA_left) << COLUMNS - 1)) & BA_path[-2]

    return nearestNeighbour2(BA_path[0:-1], final_path+[removeTrailingBits(BA_neighbours)])


def removeTrailingBits(BA):
    index = BA.index(True)
    BA.setall(False)
    BA[index] = True
    return BA
    

def getVector(grid, pose_x, pose_y):

    global ROWS
    ROWS = len(grid)
    global COLUMNS
    COLUMNS = len(grid[0])
    
    global probability_certainty
    probability_certainty = 0.9
    pose_ang = 0

    #create new bit arrays
    global BA_null_set
    BA_null_set = bitarray('0'*(ROWS*COLUMNS))
    BA_unknown = bitarray('0'*(ROWS*COLUMNS))
    BA_void = bitarray('0'*(ROWS*COLUMNS))
    BA_wall = bitarray('0'*(ROWS*COLUMNS))
    global BA_left
    BA_left = bitarray(('1'+'0'*(COLUMNS-1)) * ROWS)
    global BA_right
    BA_right = bitarray(('0'*(COLUMNS-1)+'1') * ROWS)
    global BA_up
    BA_up = bitarray('1'*COLUMNS + '0'*(COLUMNS*(ROWS-1)))
    global BA_down
    BA_down = bitarray('0'*(COLUMNS*(ROWS-1)) + '1'*COLUMNS)

    #convert grid to bit arrays ||| SUPER SLOW |||
    for index_row, row in enumerate(OCCUPENCY_GRID):
        for index_column, value in enumerate(row):
            index = index_column + (index_row * COLUMNS)
            if value < 0:
                BA_unknown[index] = 1
            elif value < probability_certainty:
                BA_void[index] = 1
            else:
                BA_wall[index] = 1

    BA_current_search =  bitarray('0'*(ROWS*COLUMNS))
    BA_current_search[pose_y*COLUMNS + pose_x] = 1

    BA_path = nearestNeighbour1(BA_unknown, BA_wall, [BA_current_search], BA_current_search)
    index = nearestNeighbour2(BA_path, [removeTrailingBits(BA_path[-1].copy())])[-2].index(True)

    node_x = index % COLUMNS
    node_y = int(index/COLUMNS)
    vector_x = node_x - pose_x
    vector_y = node_y - pose_y

    return vector_x, vector_y
