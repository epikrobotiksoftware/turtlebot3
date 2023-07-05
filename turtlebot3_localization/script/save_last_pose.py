#! /usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped
from rclpy.executors import MultiThreadedExecutor
from geometry_msgs.msg import Point
import yaml
import time
import os
from ament_index_python.packages import get_package_share_directory
from tf_transformations import euler_from_quaternion
import getpass


class AmclPoseSubscriber(Node):
    def __init__(self):
        self.amcl_data_x = None
        self.amcl_data_y = None
        self.amcl_data_yaw = None
        super().__init__('amcl_pose')
        self.subscription = self.create_subscription(
            PoseWithCovarianceStamped, '/amcl_pose', self.callback, 10)

    def callback(self, msg):
        self.amcl_data_x = msg.pose.pose.position.x
        self.amcl_data_y = msg.pose.pose.position.y
        orientation_list = [msg.pose.pose.orientation.x, msg.pose.pose.orientation.y,
                            msg.pose.pose.orientation.z, msg.pose.pose.orientation.w]
        (roll, pitch, yaw) = euler_from_quaternion(orientation_list)
        self.amcl_data_yaw = yaw
        pose = {
            "initial_pose_x": self.amcl_data_x,
            "initial_pose_y": self.amcl_data_y,
            "initial_pose_z": 0.0,
            "initial_pose_yaw": self.amcl_data_yaw,
        }

        yaml_name = "save_last_pose.yaml"
        WS = os.getenv("WS")
        META_PACKAGE_NAME = os.getenv("META_PACKAGE_NAME")
        ROBOT_NAME = os.getenv("robot_name")

        yaml_config = f"{WS}/{META_PACKAGE_NAME}/{ROBOT_NAME}_localization/config/{yaml_name}"
        # print("------------------------------------------------------------------------", yaml_config)
        with open(yaml_config, "w") as file:
            yaml.dump(pose, file)
            print("saved")
            file.close()


def main(args=None):
    rclpy.init(args=args)
    amcl_pose_subscriber = AmclPoseSubscriber()
    time.sleep(1)
    executor = MultiThreadedExecutor(2)
    executor.add_node(amcl_pose_subscriber)
    executor.spin()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
