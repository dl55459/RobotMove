import rclpy
from rclpy.node import Node
import numpy as np
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
import heapq
from eyes_msgs.msg import DetectedEntityArray, DetectedEntity
from geometry_msgs.msg import Point
from std_msgs.msg import Bool, String
#--------------------------------------------------------------------------------------------------

class PathfinderAStar(Node):
    def __init__(self):
        super().__init__('pathfinder_a_star')

        self.path_pub = self.create_publisher(Path, '/pathfinder/path', 10)

        self.entity_sub = self.create_subscription(DetectedEntityArray, '/robot_eyes/entities', self.entities_callback, 10)
        self.ready_sub = self.create_subscription(Bool, '/pathfinder/ready', self.ready_callback, 10)
        self.start_sub = self.create_subscription(String, '/pathfinder/start_point', self.startpoint_callback, 10)
        self.goal_sub = self.create_subscription(String, '/pathfinder/goal_point', self.goalpoint_callback, 10)

        self.start = ['', None, None]
        self.goal  = ['', None, None]

        self.grid = np.zeros((350, 350), dtype=np.uint8) # Work area is 350mm x 350mmm

        self.scale = 1000 # m to mm
        self.grid_ready = False

#--------------------------------------------------------------------------------------------------

    def startpoint_callback(self, msg):
        if self.start[0] != msg.data:
            self.start = [msg.data, None, None]

    def goalpoint_callback(self, msg):
        if self.goal[0] != msg.data:
            self.goal = [msg.data, None, None]

#--------------------------------------------------------------------------------------------------

    def ready_callback(self, msg):
        if (msg.data and self.grid_ready):
            self.get_logger().info('Received ready signal, making a path..')
            
            # Run A*
            start = (self.start[1], self.start[2])
            goal = (self.goal[1], self.goal[2])

            path = self.a_star(start, goal)

            if path:
                self.publish_path(path)
                self.get_logger().info(f"Path found with {len(path)} points.")
                self.get_logger().info(f"{path}")
            else:
                self.get_logger().warn("No path found.")
            
            self.grid_ready = False

#--------------------------------------------------------------------------------------------------

    def entities_callback(self, msg):

        if (self.start[0] == '' or self.goal[0] == ''):
            self.get_logger().info("Waiting for Start and Goal names...")
            return

        self.grid.fill(0) # Clear the grid

        # Robot's base area is always forbidden to enter
        # The area is 150mm x 150mm and located: centered at the bottom of the work area
        # (0-199 free, 200-349 base) X axis
        # (0-99 free, 100-249 base, 250-349 free) Y axis
        base_ul_x = 200
        base_ul_y = 100
        base_dr_x = 349
        base_dr_y = 249
        self.grid[ base_ul_y:base_dr_y+1, base_ul_x:base_dr_x+1] = 1

        for entity in msg.entities:

            # Convert R center coordinates to grid coordinates
            cx = int(entity.center.x * self.scale)
            cy = int(entity.center.y * self.scale)
            # # Clip to grid bounds, in case center is outside work area or negative
            cx = max(0, min(self.grid.shape[1] - 1, cx))
            cy = max(0, min(self.grid.shape[1] - 1, cy))

            # Is the current entity our start?
            if entity.name == self.start[0]:
                self.start[1] = cx
                self.start[2] = cy

            # Is the current entity our goal?
            elif entity.name == self.goal[0]:
                self.goal[1] = cx
                self.goal[2] = cy

            # It's an obstacle
            else:
                #Find top left and bottom right corners
                p1_x = int(entity.corner_ul.x * self.scale)
                p1_y = int(entity.corner_ul.y * self.scale)
                p2_x = int(entity.corner_dr.x * self.scale)
                p2_y = int(entity.corner_dr.y * self.scale)

                # Clip to grid bounds, in case one of the corners is outside work area or negative
                p1_x = max(0, min(self.grid.shape[1] - 1, p1_x))
                p2_x = max(0, min(self.grid.shape[1] - 1, p2_x))
                p1_y = max(0, min(self.grid.shape[0] - 1, p1_y))
                p2_y = max(0, min(self.grid.shape[0] - 1, p2_y))

                self.grid[p1_y:p2_y+1, p1_x:p2_x+1] = 1

        self.grid_ready = True
        if( (self.start[1] is None) or (self.start[2] is None) or (self.goal[1] is None) or (self.goal[2] is None)):
            self.get_logger().warn("Start and/or Goal invalid")
            self.grid_ready = False
        else:
            self.get_logger().info("Path Matrix updated")

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

    def publish_path(self, path):
        msg = Path()
        msg.header.frame_id = 'map'
        msg.header.stamp = self.get_clock().now().to_msg()

        for x, y in path:
            pose = PoseStamped()
            pose.header = msg.header
            pose.pose.position.x = x * 0.001  # Scale grid to meters
            pose.pose.position.y = y * 0.001
            pose.pose.orientation.w = 1.0
            msg.poses.append(pose)

        self.path_pub.publish(msg)

#--------------------------------------------------------------------------------------------------

def main(args=None):
    rclpy.init(args=args)
    pathfinder_a_star = PathfinderAStar()
    rclpy.spin(pathfinder_a_star)
    pathfinder_a_star.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()