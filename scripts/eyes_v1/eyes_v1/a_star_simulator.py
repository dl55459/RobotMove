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

class AStarSimulator(Node):
    def __init__(self):
        super().__init__('a_star_simulator')

        # Topics that will come from Robot Eyes Node
        self.entities_pub = self.create_publisher(DetectedEntityArray, '/robot_eyes/entities', 10)
        # Topic that will come from Kinematics Node
        self.ready_pub = self.create_publisher(Bool, '/pathfinder/ready', 10)
        self.start_pub = self.create_publisher(String, '/pathfinder/start_point', 10)
        self.goal_pub = self.create_publisher(String, '/pathfinder/goal_point', 10)

        self.sim_entities = DetectedEntityArray()
        self.make_boxes()

        self.timer = self.create_timer(1.0, self.publish_all)

#--------------------------------------------------------------------------------------------------

    def make_boxes(self):

        start = DetectedEntity()
        start.name = 'cat'
        start.center    = Point(x=0.314817, y=0.059777, z=0.0) # (314, 59)
        start.corner_ul = Point(x=0.299999, y=0.042459, z=0.0) # (299, 42)
        start.corner_dr = Point(x=0.377191, y=0.076295, z=0.0) # (377, 76)
        
        goal = DetectedEntity()
        goal.name = 'dog'
        goal.center    = Point(x=0.175938, y=0.322581, z=0.0) # (175, 322)
        goal.corner_ul = Point(x=0.151741, y=0.300000, z=0.0) # (151, 300)
        goal.corner_dr = Point(x=0.185675, y=0.333433, z=0.0) # (185, 333)

        obs1 = DetectedEntity()
        obs1.name = 'car'
        obs1.center    = Point(x=0.100000, y=0.185614, z=0.0) # (100, 185)
        obs1.corner_ul = Point(x=0.051741, y=0.111515, z=0.0) # (51, 111)
        obs1.corner_dr = Point(x=0.222222, y=0.222222, z=0.0) # (222, 222)

        self.sim_entities.entities.append(start)
        self.sim_entities.entities.append(goal)
        self.sim_entities.entities.append(obs1)

#--------------------------------------------------------------------------------------------------

    def publish_all(self):
        self.start_pub.publish(String(data='cat'))
        self.goal_pub.publish(String(data='dog'))
        self.entities_pub.publish(self.sim_entities)
        self.ready_pub.publish(Bool(data=True))
        self.ready_pub.publish(Bool(data=True))

        self.timer.cancel()

#--------------------------------------------------------------------------------------------------

def main(args=None):
    rclpy.init(args=args)
    a_star_simulator = AStarSimulator()
    rclpy.spin(a_star_simulator)
    a_star_simulator.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()