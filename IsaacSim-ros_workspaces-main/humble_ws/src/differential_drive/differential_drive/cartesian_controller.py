
#! /usr/bin/env python

import rclpy 
from rclpy.node import Node
from example_interfaces.msg import Float64 
from geometry_msgs.msg import  Pose

import numpy as np 

class CartesianController(Node):
    def __init__(self):

        self.x_position=float(0)
        self.y_position=float(0)
        self.theta_orientation=float(0)

        self.x_targhet=float(10)
        self.y_targhet=float(10)
        
        

        super().__init__("CartesianController")
        self.get_logger().info("Start Cartesian Controller")
        self.create_timer(0.01, self.timer_callback)

        self.linear_velocity_pub=self.create_publisher(Float64,"linear_velocity",10)
        self.angular_velocity_pub=self.create_publisher(Float64,"angular_velocity",10)

        #The topic /robot-position is directly published from Isaac_sim angle_targhet=np.atan2(error_x,error_y)-theta_c  #+np.pi #wryyy
        self.pose_sub= self.create_subscription(Pose, "/robot_position" ,self.pose_callback,10)
        self.targhet_sub=self.create_subscription(Pose,"/targhrt_position",self.targhet_pose_callback,10)
        
        #DEBUG
        self.count = 0 
        

    def linear_error(self,x_measured,y_measured,x_d,y_d):
        # ERROR= TARGHET-MEASURE 
        return float(np.sqrt(pow((x_d-x_measured),2)+pow((y_d-y_measured),2)))

    def angular_error(self,x_c,y_c,theta_c,x_d,y_d):

        error_x = x_d-x_c
        error_y = y_d-y_c
        
        angle_targhet=np.atan2(error_y,error_x)-theta_c #-(np.pi) #wryyy

        return float(angle_targhet)


    def pose_callback(self, pose:Pose):


        self.x_position=pose.position.x
        self.y_position=pose.position.y
        quaternion_z=0
        quaternion_w=0
        
        quaternion_x=pose.orientation.x
        quaternion_y=pose.orientation.y
        quaternion_z=pose.orientation.z
        quaternion_w=pose.orientation.w

        #conversion quaternion to euler angle 
        # yaw

        q_y_sqr=quaternion_y*quaternion_y
        q_z_sqr=quaternion_z*quaternion_z
        siny_cosp= 2*(quaternion_w*quaternion_z+quaternion_x*quaternion_y)
        cosy_cosp= +1 -2*(q_y_sqr+q_z_sqr)

        self.theta_orientation=np.atan2(siny_cosp,cosy_cosp) # the orientation of the robot is defined  as the rotetion around the z axis

        #debug

        #self.get_logger().info("Position recived "+"x :"+str( self.x_position)+" y :"+str( self.y_position)+"theta :"+str( (self.theta_orientation*180)/np.pi))

    def targhet_pose_callback(self,pose:Pose):
         
         self.x_targhet=pose.position.x
         self.y_targhet=pose.position.y
         #self.get_logger().info(" call targhet Position x:" + str(self.x_targhet) +" y : "+ str(self.y_targhet))
    
    
    def timer_callback(self):
         

        k_linear = 0.4
        k_angular = 0.8

        v,w =self.controller(self.x_targhet,self.y_targhet,k_linear,k_angular)
        
        #self.get_logger().info("Action Computed  Linear: v= "+ str(v) +" Angualr: w= " +str(w) )
        
        v_f=Float64()
        w_f=Float64()
        v_f.data=v
        w_f.data=w

        self.linear_velocity_pub.publish(v_f)
        self.angular_velocity_pub.publish(w_f)
        
        #DEbug
        #self.get_logger().info("timer Callback" + str(self.count))
        #self.count+=1
        

    def controller(self,x_desired, y_desired, k_linear ,k_angular):

        #Simple Proportional implementation 
        #v=float(k_linear)*self.linear_error(self.x_position,self.y_position,x_desired,y_desired)
        #w=float(k_angular)*self.angular_error(self.x_position,self.y_position,self.theta_orientation,x_desired,y_desired)

        #Polar Cartesian controller 
        rho=self.linear_error(self.x_position,self.y_position,x_desired,y_desired)
        gamma=self.angular_error(self.x_position,self.y_position,self.theta_orientation,x_desired,y_desired)
        delta= gamma+self.theta_orientation
        
        k1=50
        k2=50 #0.5
        k3=50

        if (rho>0.001):
            v= k1*rho*np.cos(gamma)
            w= k2*gamma +k1*((np.sin(gamma)*np.cos(gamma))/gamma)+(gamma+k3*delta)
        else:
            v=0.0
            w=0.0
        
        

        #DEBUG
        self.get_logger().info("  Linear ERROR= "+ str( self.linear_error(self.x_position,self.y_position,x_desired,y_desired) ) +" Angualr: ERROR = " +str(self.angular_error(self.x_position,self.y_position,self.theta_orientation,x_desired,y_desired)) )

        

        return v,w 
    

    

def main (args=None):
    
    rclpy.init(args=args)
    node = CartesianController()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__== '__main__':
    main()