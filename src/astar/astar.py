class Node:
    def __init__(self, coord, parent, start, goal, occupancy, width):
        self.map_width = width
        self.occupancy = occupancy
        self.coord = coord
        self.h = self.calc_h(goal, self.coord)
        if parent:
            self.parent = parent
            self.g = self.calc_g(self.coord, self.parent)
            self.f = self.h + self.g
        else:
            self.parent = None
            self.g = -1
            self.f = -1
        self.start = start
        self.goal = goal

    def __eq__(self, other):
        return self.coord == other.coord

    def __ne__(self, other):
        return self.coord != other.coord

    def __lt__(self, other):
        return self.f < other.f

    def __gt__(self, other):
        return self.f > other.f

    def __le__(self, other):
        return self.f <= other.f

    def __ge__(self, other):
        return self.f >= other.f

    def set_parent(self, parent):
        self.parent = parent

    def calc_g(self, coord, parent) -> int:
        coord_2D = get_2D(coord, self.map_width)

        parent_2D = get_2D(parent.coord, self.map_width)
        parent_g = parent.g

        if parent_2D[0] != coord_2D[0] and parent_2D[1] != coord_2D[1]:
            return parent_g + 14
        return parent_g + 10

    def update_g(self, goal, parent):
        result = self.calc_g(goal, parent)
        if result < self.g:
            self.g = result
            self.set_parent(parent)

    def calc_h(self, goal, coord) -> int:
        goal_2D = get_2D(goal, self.map_width)
        coord_2D = get_2D(coord, self.map_width)

        vertical_mv, horizontal_mv = abs(goal_2D[0] - coord_2D[0]), abs(goal_2D[1] - coord_2D[1])
        total_mv = vertical_mv + horizontal_mv
        diagonal_mv = int((total_mv - abs(vertical_mv - horizontal_mv)) / 2)
        straight_mv = total_mv - diagonal_mv*2

        return diagonal_mv * 14 + straight_mv * 10

    def update_f(self):
        if self.h+self.g < self.f:
            self.f = self.h+self.g


def find_path(grid, start, goal, width):

    grid = list(grid)
    open_nodes = []
    closed = []
    starting = Node(start, None, start, goal, grid[start], width)
    current = starting
    open_nodes.append(current)
    while True:
        if current.coord == goal:
            target = current
            break
        open_nodes.remove(current)
        closed.append(current)
        neighbours = find_neighbours(current, grid, width)
        for neighbour in neighbours:
            surrounding_walls = check_neighbours(neighbour, grid, width, 4)
            if surrounding_walls:
                #[print(f"{get_world_coord(get_2D(i, width))} wall found") for i in surrounding_walls]
                costmap_coords = get_costmap_coords(surrounding_walls, width, 4)
                #print(f"{len(costmap_coords)}\n\n")
                for coord in costmap_coords:
                    if grid[coord] != 100 and grid[coord] != -1:
                        #print(f"{get_world_coord(get_2D(coord, width))} excluded")
                        grid[coord] = 200
                continue
            elif neighbour in closed or neighbour.occupancy == 200 or neighbour.occupancy == 100:
                continue
            else:
                neighbour.update_g(current.goal, current)
                neighbour.update_f()
                if neighbour not in open_nodes:
                    open_nodes.append(neighbour)
        if not open_nodes:
            return None
        current = min(open_nodes)
    return target

def find_neighbours(node, grid, width):
    coord_2D = get_2D(node.coord, width)

    all_neighbours = []
    for i in range(-1, 2):
        for j in range(-1, 2):
            if i == 0 and j == 0:
                continue
            else:
                row, col = coord_2D[0] + i, coord_2D[1] + j
                if row <0 or row >= width:
                    continue
                if col <0 or col >= width:
                    continue
                coord_1D = get_1D(row, col, width)
                all_neighbours.append(Node(coord_1D, node, node.start, node.goal, grid[coord_1D],width))
    return all_neighbours


def row(index, width):
    return int(index / width)


def column(index, width):
    return int(index % width)


def get_2D(coord, width):
    return row(coord, width), column(coord, width)


def get_1D(row, col, width):
    result = (int(row) * width) + col
    return result


def traverse_path(node, path):
    path.append(node.coord)
    if not node.parent:
        return path
    return traverse_path(node.parent, path)


def translate_path(path, width):
    vectors_2D = []
    path_2D = [get_2D(waypoint, width) for waypoint in path]
    #print(path_2D)
    path_2D = path_2D[::-1]
    previous = path_2D[0]
    for step in path_2D[1:]:
        vectors_2D.append([step[0] - previous[0], step[1] - previous[1]])
        previous = step
 #   print(vectors_2D)
    merged = [vectors_2D[0]]
    last_vec = vectors_2D[0]
    for vec in vectors_2D[1:]:
        if vec != last_vec:
            merged.append(vec)
        else:
            merged[-1][0] += vec[0]
            merged[-1][1] += vec[1]
        last_vec = vec

    return merged

def check_neighbours(cell, grid, width, layers):
    current_layer = find_neighbours(cell, grid, width)
    if layers == 1:
        walls = []
        for node in current_layer:
            if grid[node.coord] == 100:
                walls.append(node.coord)
        return walls
    else:
        all_walls = []
        for cell in current_layer:
            if grid[cell.coord] == 100:
                all_walls += [cell.coord] + check_neighbours(cell, grid, width, layers-1)
            else:
                all_walls += check_neighbours(cell, grid, width, layers-1)
        return sorted(list(set(all_walls)))


def get_surrounding(coord, width):
    coords = []
    coord_2D = get_2D(coord, width)
    for i in range(-1, 2):
        for j in range(-1, 2):
            if i == 0 and j == 0:
                continue
            else:
                row, col = coord_2D[0] + i, coord_2D[1] + j
                if row < 0 or row >= width:
                    continue
                if col < 0 or col >= width:
                    continue
                coord_1D = get_1D(row, col, width)
                coords.append(coord_1D)
    return coords


def get_costmap_coords(walls, width, layers):
    coords = []
    for i in walls:
        coords += get_costmap(i, width, layers)
    return sorted(list(set(coords)))


def get_costmap(cell, width, layers):
    current_layer = get_surrounding(cell, width)
    if layers == 1:
        return current_layer
    else:
        all_coords = []
        for coord in current_layer:
            all_coords += get_costmap(coord, width, layers-1)
        return all_coords

def get_world_coord(vector):
    x = (vector[0] * 0.15) - 100.05
    y = (vector[1] * 0.15) - 100.05
    return [x, y]


def get_grid_coord(pose):
    x = (pose.x + 100.05) / 0.15
    y = (pose.y + 100.05) / 0.15
    return x, y


if __name__ == "__main__":
    test_grid = [
        [9, 0, 0, 0, 0],
        [0, 0, 100, 0, 0],
        [0, 0, 0, 0, 0],
        [0, 0, 0, 0, 9],
        [0, 0, 0, 0, 0]
    ]
    test_grid_1d = [9, 0, 0, 0, 0,0, 0, 100, 0, 0,0, 0, 0, 0, 0,0, 0, 0, 0, 9,0, 0, 0, 0, 0]

    test_cut = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1,
                    1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]

    test_stuff = [100, 0, 100, 100, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 100, 0, 0, 0, 0, 0, 0, 100, 0,   100, 100, 0, 100, 0, 100, 0, 0, 100, 100, 0]
    test2 = [0 for i in range(7)] + [100] + [0 for i in range(28)]

    path = find_path(test_grid_1d, 0, 19, 5)
    print(path)
    traversed = traverse_path(path, [])
    print(traversed)
    translated = translate_path(traversed, 5)
    print(translated)

    #wall_neighbours = check_neighbours(Node(15, None, 15, 0, 0, 6), test2, 6, 3)
    #print(wall_neighbours)
    #costmap_coords = get_costmap_coords(wall_neighbours, 6, 3)
    #print(costmap_coords)
    cut = 2
    cols = 10
    rows = 5

