#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import math


class LidarFilterNode(Node):
    def __init__(self):
        super().__init__('lidar_filter_node')
        # Subscriber to the /scan topic
        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10
        )
        # Publisher for the /filtered_scan topic
        self.publisher = self.create_publisher(LaserScan, '/filtered_scan', 10)
        self.get_logger().info("LidarFilterNode is up and running...")

    def scan_callback(self, msg: LaserScan):
        # Copy the original LaserScan message
        filtered_scan = LaserScan()
        filtered_scan.header = msg.header
        filtered_scan.angle_min = 0.0  # 0 degrees in radians
        filtered_scan.angle_max = math.radians(120)  # 120 degrees in radians
        filtered_scan.angle_increment = msg.angle_increment
        filtered_scan.time_increment = msg.time_increment
        filtered_scan.scan_time = msg.scan_time
        filtered_scan.range_min = msg.range_min
        filtered_scan.range_max = msg.range_max
        print("the scan topic data for unfiltered lidar is: ", msg)
        # Calculate the indices for the range of interest
        start_index = int((filtered_scan.angle_min - msg.angle_min) / msg.angle_increment)
        end_index = int((filtered_scan.angle_max - msg.angle_min) / msg.angle_increment)

        # Filter ranges and intensities
        filtered_scan.ranges = msg.ranges[start_index:end_index]
        if len(msg.intensities) > 0:
            filtered_scan.intensities = msg.intensities[start_index:end_index]
        else:
            filtered_scan.intensities = []

        # Publish the filtered scan
        self.publisher.publish(filtered_scan)
        self.get_logger().info(f"Published filtered scan with {len(filtered_scan.ranges)} ranges.")

def main(args=None):
    rclpy.init(args=args)
    node = LidarFilterNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Node terminated by user.")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
