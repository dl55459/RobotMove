import rclpy
from rclpy.node import Node
import numpy as np
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
import heapq
from eyes_msgs.msg import DetectedEntityArray
from geometry_msgs.msg import Point
from std_msgs.msg import Bool
#--------------------------------------------------------------------------------------------------

class PathfinderAStar(Node):
    def __init__(self):
        super().__init__('pathfinder_a_star')

        self.grid_ready = False

        self.grid = np.zeros((350, 350), dtype=np.uint8) # Work area is 350mm x 350mmm
        self.scale = 1000 # m to mm

        self.start = ['cat', None, None] # For now, we hard code the names
        self.goal = ['dog', None, None]  # And we ont care about their bbox, just centers

        start_t =   ['cat',  0.300000, 0.050000,   0.000333, 0.004141,   0.002581, 0.006137]
        goal_t =    ['dog',  0.300000, 0.300000,   0.007519, 0.006101,   0.009591, 0.008101]
        obstacle1 = ['bruh', 0.004500, 0.005500,   0.004573, 0.002674,   0.005274, 0.007989]
        base =      ['base', 0.275000, 0.175000,   0.200000, 0.100000,   0.349000, 0.249000]

        test_set = []
        test_set.append(start_t)
        test_set.append(goal_t)
        test_set.append(obstacle1)
        test_set.append(base)

        self.fill_grid(test_set)
        if self.grid_ready:
            start_input = (self.start[1], self.start[2])
            goal_input = (self.goal[1], self.goal[2])
            path = self.a_star(start_input, goal_input)

        if path is None:
            self.get_logger().warn("No path found.")
        else:
            self.get_logger().info(f"Start: {self.start}")
            self.get_logger().info(f"Goal: {self.goal}")
            self.get_logger().info(f"Path: {path}")

#--------------------------------------------------------------------------------------------------

    def fill_grid(self, entities):

        self.grid.fill(0) # Clear the grid

        for entity in entities:

            # Convert R center coordinates to grid coordinates
            cx = int(entity[1] * self.scale)
            cy = int(entity[2] * self.scale)
            # # Clip to grid bounds, in case center is outside work area or negative
            cx = max(0, min(self.grid.shape[1] - 1, cx))
            cy = max(0, min(self.grid.shape[1] - 1, cy))

            # Is the current entity our start?
            if entity[0] == self.start[0]:
                self.start[1] = cx
                self.start[2] = cy

            # Is the current entity our goal?
            elif entity[0] == self.goal[0]:
                self.goal[1] = cx
                self.goal[2] = cy

            # It's an obstacle
            else:
                #Find top left and bottom right corrners
                p1_x = int(entity[3] * self.scale)
                p1_y = int(entity[4] * self.scale)
                p2_x = int(entity[5] * self.scale)
                p2_y = int(entity[6] * self.scale)

                # Clip to grid bounds, in case one of the corners is outside work area or negative
                p1_x = max(0, min(self.grid.shape[1] - 1, p1_x))
                p1_y = max(0, min(self.grid.shape[0] - 1, p1_y))
                p2_x = max(0, min(self.grid.shape[1] - 1, p2_x))
                p2_y = max(0, min(self.grid.shape[0] - 1, p2_y))

                self.grid[p1_y:p2_y+1, p1_x:p2_x+1] = 1 # THIS MAKES NO FUCKING SENSE, TREAT IT AS [COLLUM, ROWS]

        self.grid_ready = True
        if( (self.start[1] == None) or (self.start[2] == None) or (self.goal[1] == None) or (self.goal[2] == None)):
            self.get_logger().warn("Start and/or Goal invalid")
            self.grid_ready = False

#--------------------------------------------------------------------------------------------------

    def heuristic(self, a, b):
        dx = abs(a[0] - b[0])
        dy = abs(a[1] - b[1])
        return max(dx, dy)  # Chebyshev distance

#--------------------------------------------------------------------------------------------------

    def a_star(self, start, goal):
        neighbors = [(0, 1), (1, 1), (1, 0), (1, -1), (0, -1), (-1, -1), (-1, 0), (-1, 1)]
        open_set = []
        heapq.heappush(open_set, (0 + self.heuristic(start, goal), 0, start))
        came_from = {}
        g_score = {start: 0}

        while open_set:
            _, cost, current = heapq.heappop(open_set)

            if current == goal:
                path = []
                while current in came_from:
                    path.append(current)
                    current = came_from[current]
                path.append(start)
                path.reverse()
                return path

            for dx, dy in neighbors:
                neighbor = (current[0] + dx, current[1] + dy)
                if (0 <= neighbor[0] < self.grid.shape[1] and
                    0 <= neighbor[1] < self.grid.shape[0] and
                    self.grid[neighbor[1]][neighbor[0]] == 0):

                    step_cost = 1.4 if dx != 0 and dy != 0 else 1
                    tentative_g = g_score[current] + step_cost
                    
                    if neighbor not in g_score or tentative_g < g_score[neighbor]:
                        came_from[neighbor] = current
                        g_score[neighbor] = tentative_g
                        f = tentative_g + self.heuristic(neighbor, goal)
                        heapq.heappush(open_set, (f, tentative_g, neighbor))

        return None

#--------------------------------------------------------------------------------------------------

def main(args=None):
    rclpy.init(args=args)
    pathfinder_a_star = PathfinderAStar()
    rclpy.spin(pathfinder_a_star)
    pathfinder_a_star.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()