import numpy as np


class GridMap:
    def __init__(self, width=20, height=20, resolution=1.0):
        self.width = width
        self.height = height
        self.resolution = resolution

        self.grid = np.zeros((height, width))
        self.obstacles = []


    # ADD RECTANGULAR OBSTACLE
    def add_obstacle_rect(self, top_left, bottom_right):
        x1, y1 = top_left
        x2, y2 = bottom_right

        # Clip to grid bounds (VERY IMPORTANT)
        x1 = max(0, min(self.width, x1))
        x2 = max(0, min(self.width, x2))
        y1 = max(0, min(self.height, y1))
        y2 = max(0, min(self.height, y2))

        self.grid[x1:x2, y1:y2] = 1

        self.obstacles.append(((x1,y1), (x2,y2)))

    # CHECK IF CELL IS FREE
    def is_free(self, x, y):
        # Out of bounds = occupied
        if x < 0 or x >= self.width or y < 0 or y >= self.height:
            return False

        return self.grid[x, y] == 0

    # OPTIONAL: DEBUG PRINT
    def show(self):
        print(self.grid)