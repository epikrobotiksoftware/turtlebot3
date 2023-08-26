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

import std_msgs.msg

class OccupancyGridSubscriber(LifecycleNode):
    def __init__(self):
        super().__init__('liveMap')

        self.image_base64 = ""
        self.timer = None
        self.publisher_ = None
        self.subscription = None

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
        if self.image_base64 == "":
            return
        msg = String()
        msg.data = self.image_base64
        self.publisher_.publish(msg)

    def on_configure(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info("on_configure() is called.")
        self.publisher_ = self.create_publisher(String, 'liveMap', 10)
        self.timer = self.create_timer(0.1, self.publish_image)  # Adjust the rate as needed
        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info("on_activate() is called.")
        self.subscription = self.create_subscription(
            OccupancyGrid,
            '/map',
            self.occupancy_grid_callback,
            10
        )
        return TransitionCallbackReturn.SUCCESS

    def on_deactivate(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info("on_deactivate() is called.")
        self.destroy_subscription(self.subscription)
        return TransitionCallbackReturn.SUCCESS

    def on_cleanup(self, state: State) -> TransitionCallbackReturn:
        if self.timer:
            self.timer.cancel()
        if self.publisher_:
            self.destroy_publisher(self.publisher_)
        self.image_base64 = ""
        self.subscription = None
        self.get_logger().info('on_cleanup() is called.')
        return TransitionCallbackReturn.SUCCESS

    def on_shutdown(self, state: State) -> TransitionCallbackReturn:
        if self.timer:
            self.timer.cancel()
        if self.publisher_:
            self.destroy_publisher(self.publisher_)
        self.get_logger().info('on_shutdown() is called.')
        self.destroy_node()  # Call destroy_node() explicitly
        return TransitionCallbackReturn.SUCCESS
    
    def destroy_node(self):
        if self.timer:
            self.timer.cancel()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    executor = rclpy.executors.SingleThreadedExecutor()
    node = OccupancyGridSubscriber()
    executor.add_node(node)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()