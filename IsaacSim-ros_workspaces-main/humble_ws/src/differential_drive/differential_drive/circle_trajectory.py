import rclpy 
from rclpy.node import Node

import numpy as np 

class CircleTrajectory(Node):
    def __init__(self):
        super().__init__("circle_trajectory")
        self.get_logger().info("Circle Trajectory Node Started")
        self.radius=0;
        self.x_center=0;
        self.y_center=0;
    

    def next_point():
        pass
        

def main (args=None):
    
    rclpy.init(args=args)
    node = CircleTrajectory()
    rclpy.spin(node)
    rclpy.shutdown()


