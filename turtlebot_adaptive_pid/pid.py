#!/usr/bin/env python3
'''
Adaptive PID Controller for Turtlesim
Author: Bibek
'''

import rclpy
from rclpy.node import Node
from turtlesim.msg import Pose
from geometry_msgs.msg import Twist
import math


class PID(Node):
    def __init__(self):
        super().__init__("pid_controller")
        
        # Turtle state (unknown initially)
        self.turtle_x = None
        self.turtle_y = None
        self.turtle_theta = None
        
        # Goal position
        self.goal_x = 9.0
        self.goal_y = 9.0
        
        # Computed values
        self.goal_distance = 0.0
        self.goal_theta = 0.0
        
        # Thresholds
        self.linear_threshold = 0.1    # 0.1 units accuracy
        self.angular_threshold = math.radians(5)  # 5 degrees
        
        # PID parameters - Linear
        self.Kp_linear = 1.0
        self.Ki_linear = 0.0
        self.Kd_linear = 0.1
        
        # PID parameters - Angular
        self.Kp_angular = 4.0
        self.Ki_angular = 0.0
        self.Kd_angular = 0.5
        
        # PID state variables - Linear
        self.error_linear = 0.0
        self.prev_error_linear = 0.0
        self.integral_linear = 0.0
        
        # PID state variables - Angular
        self.error_angular = 0.0
        self.prev_error_angular = 0.0
        self.integral_angular = 0.0
        
        # Control outputs
        self.output_linear = 0.0
        self.output_angular = 0.0
        
        # Create subscriber to turtle pose
        self.pose_sub = self.create_subscription(
            Pose,
            '/turtle1/pose',
            self.pose_callback,
            10
        )
        
        # Create publisher for velocity commands
        self.vel_pub = self.create_publisher(
            Twist,
            '/turtle1/cmd_vel',
            10
        )
        
        self.get_logger().info("PID Controller Node Started!")
        self.get_logger().info(f"Goal: ({self.goal_x}, {self.goal_y})")
        
    
    def pose_callback(self, msg):
        """Called every time turtle pose is published (~60 Hz)"""
        # Update current pose
        self.turtle_x = msg.x
        self.turtle_y = msg.y
        self.turtle_theta = msg.theta
        
        # Calculate distance to goal
        self.goal_distance = self.calculate_distance(
            self.turtle_x, self.turtle_y,
            self.goal_x, self.goal_y
        )
        
        # Calculate desired heading
        self.goal_theta = self.calculate_angle(
            self.turtle_x, self.turtle_y,
            self.goal_x, self.goal_y
        )
        
        # Calculate angular error (wrapped to [-pi, pi])
        self.error_angular = self.wrap_angle(
            self.goal_theta - self.turtle_theta
        )
        
        # Calculate linear error
        self.error_linear = self.goal_distance
        
        # Control logic (state machine)
        if abs(self.error_angular) > self.angular_threshold:
            # State 1: Rotate to face goal
            self.output_angular = self.compute_angular_pid()
            self.output_linear = 0.0
            
        elif self.goal_distance > self.linear_threshold:
            # State 2: Move forward toward goal
            self.output_linear = self.compute_linear_pid()
            self.output_angular = 0.0
            
        else:
            # State 3: Goal reached!
            self.output_linear = 0.0
            self.output_angular = 0.0
            self.get_logger().info("Goal Reached!", once=True)
        
        # Publish velocity command
        self.publish_velocity()
        
    
    def compute_linear_pid(self):
        """Calculate linear velocity using PID"""
        # Proportional term
        p_term = self.Kp_linear * self.error_linear
        
        # Integral term
        self.integral_linear += self.error_linear
        i_term = self.Ki_linear * self.integral_linear
        
        # Derivative term
        derivative = self.error_linear - self.prev_error_linear
        d_term = self.Kd_linear * derivative
        
        # Update previous error
        self.prev_error_linear = self.error_linear
        
        # Calculate output
        output = p_term + i_term + d_term
        
        # Saturate output
        output = self.saturate(output, -2.0, 2.0)
        
        return output
    
    
    def compute_angular_pid(self):
        """Calculate angular velocity using PID"""
        # Proportional term
        p_term = self.Kp_angular * self.error_angular
        
        # Integral term
        self.integral_angular += self.error_angular
        i_term = self.Ki_angular * self.integral_angular
        
        # Derivative term
        derivative = self.error_angular - self.prev_error_angular
        d_term = self.Kd_angular * derivative
        
        # Update previous error
        self.prev_error_angular = self.error_angular
        
        # Calculate output
        output = p_term + i_term + d_term
        
        # Saturate output
        output = self.saturate(output, -2.0, 2.0)
        
        return output
    
    
    def publish_velocity(self):
        """Publish velocity command to turtle"""
        msg = Twist()
        msg.linear.x = self.output_linear
        msg.angular.z = self.output_angular
        self.vel_pub.publish(msg)
    
    
    def calculate_distance(self, x1, y1, x2, y2):
        """Euclidean distance between two points"""
        return math.sqrt((x2 - x1)**2 + (y2 - y1)**2)
    
    
    def calculate_angle(self, x1, y1, x2, y2):
        """Angle from (x1,y1) to (x2,y2)"""
        return math.atan2(y2 - y1, x2 - x1)
    
    
    def wrap_angle(self, angle):
        """Wrap angle to [-pi, pi]"""
        return math.atan2(math.sin(angle), math.cos(angle))
    
    
    def saturate(self, value, min_val, max_val):
        """Clamp value between min and max"""
        return max(min_val, min(value, max_val))


def main(args=None):
    # Initialize ROS 2
    rclpy.init(args=args)
    
    # Create node
    pid_controller = PID()
    
    # Spin (keep node running)
    rclpy.spin(pid_controller)
    
    # Cleanup
    pid_controller.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
