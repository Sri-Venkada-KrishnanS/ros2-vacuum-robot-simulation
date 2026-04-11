#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import math

class VacuumLogic(Node):
    def __init__(self):
        super().__init__('vacuum_logic')
        
        # Enforce sim_time so time-based logic strictly adheres to Gazebo ticks
        self.set_parameters([rclpy.parameter.Parameter('use_sim_time', rclpy.Parameter.Type.BOOL, True)])
        
        # Publisher to control the robot
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # Subscribe to Lidar scan
        self.subscription_scan = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10)
            
        from nav_msgs.msg import Odometry
        self.subscription_odom = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10)
        
        # Control loop at 10 Hz
        self.timer = self.create_timer(0.1, self.timer_callback)
        
        # State machine definitions
        # 0: Move Forward
        # 1: Turn 90 Degrees (First Part of U-turn)
        # 2: Move Forward slightly (Offset for next row)
        # 3: Turn 90 Degrees (Second Part of U-turn)
        self.state = 0
        self.turn_direction = 1  # 1 for left, -1 for right
        self.target_time = 0.0
        
        self.front_dist = 10.0
        self.current_yaw = 0.0
        self.target_yaw = 0.0
        self.twist_msg = Twist()
        
        self.get_logger().info('Vacuum logic node started. Sweeping area...')
        
    def odom_callback(self, msg):
        q = msg.pose.pose.orientation
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        self.current_yaw = math.atan2(siny_cosp, cosy_cosp)

    def scan_callback(self, msg):
        ranges = msg.ranges
        num_ranges = len(ranges)
        
        # Assuming typical 360 degree lidar where middle is front
        # Calculate indexing
        front_idx = num_ranges // 2
        window = max(1, num_ranges // 12)  # approx 30 degrees window
        
        front_slice = ranges[front_idx - window : front_idx + window]
        
        # Filter valid measurements
        valid_ranges = [r for r in front_slice if 0.1 < r < 20.0 and not math.isinf(r) and not math.isnan(r)]
        
        if valid_ranges:
            self.front_dist = min(valid_ranges)
        else:
            self.front_dist = 10.0

    def normalize_angle(self, angle):
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle

    def timer_callback(self):
        current_time = self.get_clock().now().nanoseconds / 1e9
        
        if self.state == 0:
            # Moving forward until an obstacle is encountered
            if self.front_dist < 0.6:
                self.get_logger().info('Obstacle detected! Initiating lawnmower turn.')
                self.state = 1
                # Calculate the NEXT absolute ideal angle based on perfect 90-degree increments
                self.target_yaw = self.normalize_angle(self.target_yaw + (math.pi / 2.0) * self.turn_direction)
                self.twist_msg.linear.x = 0.0
                self.twist_msg.angular.z = 0.5 * self.turn_direction
            else:
                self.twist_msg.linear.x = 0.3
                # Active heading correction to eliminate drift and ensure perfect straight lines
                yaw_diff = self.normalize_angle(self.target_yaw - self.current_yaw)
                self.twist_msg.angular.z = max(min(0.8 * yaw_diff, 0.3), -0.3)
                
        elif self.state == 1:
            # Turning 90 degrees
            yaw_diff = self.normalize_angle(self.target_yaw - self.current_yaw)
            # Tighter tolerance (0.01 rad is roughly 0.5 degrees vs old 0.05 rad which was 2.8 deg)
            if abs(yaw_diff) < 0.01:
                # Turn finished
                self.state = 2
                self.target_time = current_time + 1.5 # Move along the wall for 1.5s
                self.twist_msg.linear.x = 0.2
                self.twist_msg.angular.z = 0.0
            else:
                self.twist_msg.linear.x = 0.0
                # Proportional turn
                self.twist_msg.angular.z = max(min(1.5 * yaw_diff, 0.5), -0.5)
                # Ensure minimum speed to prevent stalling
                if abs(self.twist_msg.angular.z) < 0.1:
                    self.twist_msg.angular.z = 0.1 if yaw_diff > 0 else -0.1
                
        elif self.state == 2:
            # Offset movement
            if current_time >= self.target_time or self.front_dist < 0.4:
                self.state = 3
                # Calculate the NEXT absolute ideal angle based on perfect 90-degree increments
                self.target_yaw = self.normalize_angle(self.target_yaw + (math.pi / 2.0) * self.turn_direction)
                self.twist_msg.linear.x = 0.0
                self.twist_msg.angular.z = 0.5 * self.turn_direction
            else:
                self.twist_msg.linear.x = 0.2
                # Active heading correction while moving offset
                yaw_diff = self.normalize_angle(self.target_yaw - self.current_yaw)
                self.twist_msg.angular.z = max(min(0.8 * yaw_diff, 0.3), -0.3)
                
        elif self.state == 3:
            # Second turn to complete U-turn
            yaw_diff = self.normalize_angle(self.target_yaw - self.current_yaw)
            if abs(yaw_diff) < 0.01:
                self.state = 0
                # Flip the turn direction for the next wall
                self.turn_direction *= -1
                self.twist_msg.linear.x = 0.3
                self.twist_msg.angular.z = 0.0
            else:
                self.twist_msg.linear.x = 0.0
                self.twist_msg.angular.z = max(min(1.5 * yaw_diff, 0.5), -0.5)
                if abs(self.twist_msg.angular.z) < 0.1:
                    self.twist_msg.angular.z = 0.1 if yaw_diff > 0 else -0.1
                
        # Send velocity commands
        self.publisher_.publish(self.twist_msg)

def main(args=None):
    rclpy.init(args=args)
    node = VacuumLogic()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
