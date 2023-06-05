#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan

class LaserScanAnalyzer(Node):
    def __init__(self):
        super().__init__('laser_scan_analyzer')
        self.scan_subscriber = self.create_subscription(
            LaserScan,
            'scan',
            self.scan_callback,
            10
        )
        self.scan_publisher = self.create_publisher(
            LaserScan,
            'scan_filtered',
            10
        )

    def scan_callback(self, msg):
        start_angle = msg.angle_min
        end_angle = start_angle + 35.0 * (3.14159 / 180.0)
        angle_increment = msg.angle_increment

        start_index = int((start_angle - msg.angle_min) / angle_increment)
        end_index = int((end_angle - msg.angle_min) / angle_increment)

        filtered_scan = LaserScan()
        filtered_scan.header = msg.header
        filtered_scan.angle_min = start_angle
        filtered_scan.angle_max = end_angle
        filtered_scan.angle_increment = angle_increment
        filtered_scan.range_min = msg.range_min
        filtered_scan.range_max = msg.range_max
        filtered_scan.ranges = msg.ranges[start_index:end_index+1]

        # Publish the filtered scan data on a new topic
        self.scan_publisher.publish(filtered_scan)

def main(args=None):
    rclpy.init(args=args)
    laser_scan_analyzer = LaserScanAnalyzer()
    rclpy.spin(laser_scan_analyzer)
    laser_scan_analyzer.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
