#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import String  # Add this import
from PIL import Image as PILImage
import io
import base64
from rclpy.timer import Timer


class OccupancyGridSubscriber(Node):
    def __init__(self):
        super().__init__('occupancy_grid_subscriber')
        # Create publisher for 'live_map' topic
        self.publisher_ = self.create_publisher(String, 'live_map', 10)

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

    def destroy_node(self):
        self.timer.cancel()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    occupancy_grid_subscriber = OccupancyGridSubscriber()
    try:
        rclpy.spin(occupancy_grid_subscriber)
    finally:
        occupancy_grid_subscriber.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

# test