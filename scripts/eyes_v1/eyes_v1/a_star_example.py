import rclpy
from rclpy.node import Node
import numpy as np
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path

import heapq


class AStarPlanner(Node):
    def __init__(self):
        super().__init__('a_star_planner')
        self.path_pub = self.create_publisher(Path, 'a_star_path', 10)

        # Example setup
        self.grid = np.zeros((20, 20), dtype=np.uint8)
        self.add_obstacle(5, 5, 10, 6)
        self.add_obstacle(10, 8, 12, 15)

        start = (0, 0)
        goal = (19, 19)
        path = self.a_star(start, goal)

        if path:
            self.publish_path(path)
            self.get_logger().info(f"Path found with {len(path)} points.")
        else:
            self.get_logger().warn("No path found.")

    def add_obstacle(self, x1, y1, x2, y2):
        self.grid[y1:y2+1, x1:x2+1] = 1

    def heuristic(self, a, b):
        return abs(a[0] - b[0]) + abs(a[1] - b[1])

    def a_star(self, start, goal):
        neighbors = [(0, 1), (1, 0), (-1, 0), (0, -1)]
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

                    tentative_g = g_score[current] + 1
                    if neighbor not in g_score or tentative_g < g_score[neighbor]:
                        came_from[neighbor] = current
                        g_score[neighbor] = tentative_g
                        f = tentative_g + self.heuristic(neighbor, goal)
                        heapq.heappush(open_set, (f, tentative_g, neighbor))

        return None

    def publish_path(self, path):
        msg = Path()
        msg.header.frame_id = 'map'
        msg.header.stamp = self.get_clock().now().to_msg()

        for x, y in path:
            pose = PoseStamped()
            pose.header = msg.header
            pose.pose.position.x = x * 0.05  # scale grid to meters
            pose.pose.position.y = y * 0.05
            pose.pose.orientation.w = 1.0
            msg.poses.append(pose)

        self.path_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = AStarPlanner()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
