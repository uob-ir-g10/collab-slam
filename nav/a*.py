class Node:
    def __init__(self, coord, parent, start, goal, occupancy):
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
        coord_2D = get_2D(coord)

        parent_2D = get_2D(parent.coord)
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
        goal_2D = get_2D(goal)
        coord_2D = get_2D(coord)

        vertical_mv, horizontal_mv = abs(goal_2D[0] - coord_2D[0]), abs(goal_2D[1] - coord_2D[1])
        total_mv = vertical_mv + horizontal_mv
        diagonal_mv = int((total_mv - abs(vertical_mv - horizontal_mv)) / 2)
        straight_mv = total_mv - diagonal_mv*2

        return diagonal_mv * 14 + straight_mv * 10

    def update_f(self):
        if self.h+self.g < self.f:
            self.f = self.h+self.g


def find_path(grid, start, goal):
    start_index = grid.index(start)
    goal_index = grid.index(goal)

    open_nodes = []
    closed = []
    starting = Node(start_index, None, start_index, goal_index, grid[start_index])

    current = starting
    open_nodes.append(current)
    target = None
    while True:
        if current.coord == goal_index:
            target = current
            break
        open_nodes.remove(current)
        closed.append(current)
        neighbours = find_neighbours(current, grid)
        for neighbour in neighbours:
            if neighbour.occupancy == 1 or neighbour in closed:
                continue
            else:
                neighbour.update_g(current.goal, current)
                neighbour.update_f()
                if neighbour not in open_nodes:
                    open_nodes.append(neighbour)

        current = min(open_nodes)
    return target

def find_neighbours(node, grid):
    coord_2D = get_2D(node.coord)
    all_neighbours = []
    for i in range(-1, 2):
        for j in range(-1, 2):
            if i == 0 and j == 0:
                continue
            else:
                row, col = coord_2D[0] + i, coord_2D[1] + j
                if row <0 or row > 5:
                    continue
                if col <0 or col > 10:
                    continue
                coord_1D = get_1D(row, col)
                all_neighbours.append(Node(coord_1D, node, node.start, node.goal, grid[coord_1D]))
    return all_neighbours


def row(index):
    return int(index / 10)


def column(index):
    return int(index % 10)


def get_2D(coord):
    return row(coord), column(coord)


def get_1D(row, col):
    return 10*row + col


def traverse_path(node, path):
    path.append(node.coord)
    if not node.parent:
        return path
    return traverse_path(node.parent, path)


def translate_path(path):
    vectors_2D = []
    path_2D = [get_2D(waypoint) for waypoint in path]
    path_2D = path_2D[::-1]
    previous = path_2D[0]
    for step in path_2D[1:]:
        vectors.append([step[0] - previous[0], step[1] - previous[1]])
        previous = step

    return vectors_2D


if __name__ == "__main__":
    test_grid = [
        [0, 0, 9, 0, 0, 0, 0, 0, 0, 0],
        [0, 0, 0, 1, 1, 0, 0, 1, 0, 1],
        [0, 0, 0, 1, 1, 0, 0, 0, 0, 1],
        [1, 0, 0, 0, 1, 0, 7, 1, 0, 1],
        [0, 1, 0, 0, 0, 1, 1, 0, 0, 1]
    ]
    test_grid_1d = [0, 0, 9, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 0, 0, 1, 0, 1, 0, 0, 0, 1,
                    1, 0, 0, 0, 0, 1, 1, 0, 0, 0, 1, 0, 7, 1, 0, 1, 0, 1, 0, 0, 0, 1, 1, 0, 0, 1]

    start = test_grid_1d.index(9)
    goal = test_grid_1d.index(7)

    goal = find_path(test_grid_1d, 9, 7)
    path = traverse_path(goal, [])
    vectors = translate_path(path)
    print(f"{path}\n{vectors}")
