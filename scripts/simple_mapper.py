#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid, Path
from geometry_msgs.msg import TransformStamped, PoseStamped
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
from tf2_ros.transform_listener import TransformListener
from rclpy.qos import QoSProfile, QoSDurabilityPolicy
import numpy as np
import cv2
import math

class SimpleMapper(Node):
    def __init__(self):
        super().__init__('simple_mapper')
        
        self.map_res = 0.05
        self.map_size = 400 # 20m x 20m
        self.map_data = np.full((self.map_size, self.map_size), -1, dtype=np.int8)
        
        # Origin of the map is at center
        self.origin_x = - (self.map_size * self.map_res) / 2.0
        self.origin_y = - (self.map_size * self.map_res) / 2.0
        
        self.prev_cx = None
        self.prev_cy = None
        
        qos_profile = QoSProfile(
            depth=1,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL
        )
        self.pub_map = self.create_publisher(OccupancyGrid, '/map', qos_profile)
        
        # Publisher for the red line path
        self.pub_path = self.create_publisher(Path, '/robot_path', 10)
        self.path_msg = Path()
        self.path_msg.header.frame_id = 'map'
        
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        self.tf_static_broadcaster = StaticTransformBroadcaster(self)
        self.publish_static_tf()
        
        self.sub_scan = self.create_subscription(LaserScan, '/scan', self.scan_cb, 10)
        
        self.timer = self.create_timer(1.0, self.publish_map)
        self.get_logger().info('Simple Mapper started.')
        
    def publish_static_tf(self):
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'map'
        t.child_frame_id = 'odom'
        t.transform.translation.x = 0.0
        t.transform.translation.y = 0.0
        t.transform.translation.z = 0.0
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = 0.0
        t.transform.rotation.w = 1.0
        self.tf_static_broadcaster.sendTransform(t)

    def scan_cb(self, msg):
        try:
            # We want the transform from map/odom to lidar_link
            t = self.tf_buffer.lookup_transform('odom', msg.header.frame_id, rclpy.time.Time())
        except TransformException as ex:
            self.get_logger().debug(f'Could not transform: {ex}')
            return
            
        # Extract robot pose
        rx = t.transform.translation.x
        ry = t.transform.translation.y
        qx = t.transform.rotation.x
        qy = t.transform.rotation.y
        qz = t.transform.rotation.z
        qw = t.transform.rotation.w
        
        # yaw from quaternion
        siny_cosp = 2.0 * (qw * qz + qx * qy)
        cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        
        # Robot grid cell
        cx = int((rx - self.origin_x) / self.map_res)
        cy = int((ry - self.origin_y) / self.map_res)
        
        angles = np.linspace(msg.angle_min, msg.angle_max, len(msg.ranges))
        
        for i, r in enumerate(msg.ranges):
            if r < msg.range_min or np.isinf(r) or np.isnan(r):
                continue
                
            angle = yaw + angles[i]
            hit_r = min(r, 9.9)
            hx = rx + hit_r * math.cos(angle)
            hy = ry + hit_r * math.sin(angle)
            
            hx_cell = int((hx - self.origin_x) / self.map_res)
            hy_cell = int((hy - self.origin_y) / self.map_res)
            
            # Draw free space line
            # cv2.line requires integer coordinates
            cv2.line(self.map_data, (cx, cy), (hx_cell, hy_cell), 0, 1)
            
            # Draw obstacle
            if r <= msg.range_max and hit_r == r:
                if 0 <= hx_cell < self.map_size and 0 <= hy_cell < self.map_size:
                    # Thick points for better visualization
                    cv2.circle(self.map_data, (hx_cell, hy_cell), 1, 100, -1)

        # Draw the continuous trail of the robot
        if self.prev_cx is not None and self.prev_cy is not None:
            # 50 is a distinct grey color for the path
            cv2.line(self.map_data, (self.prev_cx, self.prev_cy), (cx, cy), 50, 2)
        
        self.prev_cx = cx
        self.prev_cy = cy
        
        # Continually add to and publish the path
        # Only add a new pose if we have moved sufficiently (e.g. 5cm) to save memory
        if not hasattr(self, 'last_path_x') or math.hypot(rx - self.last_path_x, ry - self.last_path_y) > 0.05:
            pose = PoseStamped()
            pose.header.stamp = msg.header.stamp
            pose.header.frame_id = 'map'
            pose.pose.position.x = rx
            pose.pose.position.y = ry
            pose.pose.position.z = 0.05
            pose.pose.orientation.x = qx
            pose.pose.orientation.y = qy
            pose.pose.orientation.z = qz
            pose.pose.orientation.w = qw
            
            self.path_msg.poses.append(pose)
            self.path_msg.header.stamp = msg.header.stamp
            self.pub_path.publish(self.path_msg)
            
            self.last_path_x = rx
            self.last_path_y = ry

    def publish_map(self):
        grid = OccupancyGrid()
        grid.header.stamp = self.get_clock().now().to_msg()
        grid.header.frame_id = 'map'
        grid.info.resolution = self.map_res
        grid.info.width = self.map_size
        grid.info.height = self.map_size
        
        grid.info.origin.position.x = self.origin_x
        grid.info.origin.position.y = self.origin_y
        grid.info.origin.position.z = 0.0
        grid.info.origin.orientation.w = 1.0
        
        grid.data = self.map_data.flatten().tolist()
        self.pub_map.publish(grid)

def main(args=None):
    rclpy.init(args=args)
    mapper = SimpleMapper()
    rclpy.spin(mapper)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
