
#! /usr/bin/env python

import rclpy 
from rclpy.node import Node

import numpy as np 

class CartesianController(Node):
    def __init__(self):
        super().__init__("CartesianController")
        self.get_logger().info("Start Cartesian Controller")
        self.create_timer(1.1, self.timer_callback)
        self.count = 0 
        

    def timer_callback(self):
        
        self.get_logger().info("timer Callback" + str(self.count))
        self.count+=1 
    
    def cartesian_controller(x, y, theta, k1 ,k2):

        v= -k1 * (x *np.cos(theta)+ y* np.sin(theta))
        w= k2 * (np.atan2(y,x)+np.pi - theta)

        return v,w 


def main (args=None):
    
    rclpy.init(args=args)
    node = CartesianController()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__== '__main__':
    main()