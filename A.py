class SearchAlgorithm:
    delta = [[-1, 0],  # go up
             [0, -1],  # go left
             [1, 0],  # go down
             [0, 1]]  # go right
    delta_name = ['^', '<', 'v', '>']

    def __init__(self, grid, heuristic=None, cost=1):
        self.grid = grid
        self.cost = cost
        if heuristic is None:
            self.heuristic = [[0] * len(self.grid[0])] * len(self.grid)
        else:
            self.heuristic = heuristic

    def search_path(self, init, goal):
        # Keeps track of the fields already visited
        closed = [[0 for col in range(len(self.grid[0]))] for row in range(len(self.grid))]
        closed[init[0]][init[1]] = 1

        # Keeps track in which order the fields are expanded
        expand = [[-1 for col in range(len(self.grid[0]))] for row in range(len(self.grid))]
        expand_list = []

        # Keeps track of action to reach the current position
        action = [[-1 for col in range(len(self.grid[0]))] for row in range(len(self.grid))]

        x = init[0]
        y = init[1]
        g = 0
        h = self.heuristic[x][y]
        f = g + h
        open_fields = [[f, g, h, x, y]]

        found = False  # flag that is set when search is complete
        resign = False  # flag set if we can't find expand
        count = 0

        while not found and not resign:
            if len(open_fields) == 0:
                resign = True
            else:
                open_fields.sort()
                open_fields.reverse()
                next = open_fields.pop()
                x = next[3]
                y = next[4]
                g = next[1]
                expand[x][y] = count
                expand_list.append([x, y])
                count += 1

                if x == goal[0] and y == goal[1]:
                    found = True
                else:
                    for i in range(len(self.delta)):
                        x2 = x + self.delta[i][0]
                        y2 = y + self.delta[i][1]
                        if x2 >= 0 and x2 < len(self.grid) and y2 >= 0 and y2 < len(self.grid[0]):
                            if closed[x2][y2] == 0 and self.grid[x2][y2] == 0:
                                g2 = g + self.cost
                                h2 = self.heuristic[x2][y2]
                                f2 = g2 + h2
                                open_fields.append([f2, g2, h2, x2, y2])
                                closed[x2][y2] = 1
                                action[x2][y2] = self.delta_name[i]
        points = []
        actions = []
        if not resign:
            while x != init[0] or y != init[1]:
                a = action[x][y]
                for i in range(len(self.delta_name)):
                    if a == self.delta_name[i]:
                        points.append((x, y))
                        actions.append(action[x][y])
                        x -= self.delta[i][0]
                        y -= self.delta[i][1]

            points.reverse()
            actions.reverse()
            expand_list.reverse()
        return points, actions, expand_list
