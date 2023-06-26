#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import String
from PIL import Image as PILImage
import io
import base64
from rclpy.timer import Timer
from rclpy.lifecycle import LifecycleNode
from rclpy.lifecycle import State
from rclpy.lifecycle import TransitionCallbackReturn

from typing import Optional

import std_msgs.msg

class OccupancyGridSubscriber(LifecycleNode):
    def __init__(self):
        super().__init__('liveMap')

        # Create publisher for 'live_map' topic
        self.publisher_ = self.create_publisher(String, 'liveMap', 10)

        self.subscription = self.create_subscription(
            OccupancyGrid,
            '/map',
            self.occupancy_grid_callback,
            10
        )
        self.image_base64 = ""
        self.timer = self.create_timer(
            0.1, self.publish_image)  # Adjust the rate as needed

    def occupancy_grid_callback(self, msg):
        # Retrieve occupancy grid properties
        width = msg.info.width
        height = msg.info.height
        data = msg.data

        # Create PIL image from occupancy grid data
        image = PILImage.new('L', (width, height))
        for y in range(height):
            for x in range(width):
                index = y * width + x
                value = data[index]
                inverted_value = 255 if value == 0 else 0
                image.putpixel((x, y), inverted_value)

        # Convert PIL image to Base64 string
        buffered = io.BytesIO()
        image.save(buffered, format='JPEG')
        self.image_base64 = base64.b64encode(
            buffered.getvalue()).decode('utf-8')

    def publish_image(self):
        # Publish the Base64 encoded image as a string
        msg = String()
        msg.data = self.image_base64
        self.publisher_.publish(msg)

    def on_configure(self, state: State) -> TransitionCallbackReturn:
        # Configure the node, after a configuring transition is requested.
        self.get_logger().info("on_configure() is called.")
        self._pub = self.create_lifecycle_publisher(std_msgs.msg.String, "lifecycle_chatter", 10)
        

        self.get_logger().info("on_configure() is called.")
        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state: State) -> TransitionCallbackReturn:
        # Activate the node, after an activating transition is requested.
        self.get_logger().info("on_activate() is called.")
        return super().on_activate(state)

    def on_deactivate(self, state: State) -> TransitionCallbackReturn:
        # Deactivate the node, after a deactivating transition is requested.
        self.get_logger().info("on_deactivate() is called.")
        return super().on_deactivate(state)

    def on_cleanup(self, state: State) -> TransitionCallbackReturn:
        # Cleanup the node, after a cleaning-up transition is requested.
        self.destroy_timer(self.timer)
        self.destroy_publisher(self._pub)

        self.get_logger().info('on_cleanup() is called.')
        return TransitionCallbackReturn.SUCCESS

    def on_shutdown(self, state: State) -> TransitionCallbackReturn:
        # Shutdown the node, after a shutting-down transition is requested.
        self.destroy_timer(self.timer)
        self.destroy_publisher(self._pub)

        self.get_logger().info('on_shutdown() is called.')
        return TransitionCallbackReturn.SUCCESS
    
    def destroy_node(self):
        self.timer.cancel()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    executor = rclpy.executors.SingleThreadedExecutor()
    node = OccupancyGridSubscriber()
    executor.add_node(node)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt):
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
