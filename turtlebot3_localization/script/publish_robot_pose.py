#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped
from tf_transformations import euler_from_quaternion
import os 
import yaml

class MyNode(Node):
    def __init__(self):
        super().__init__('slamtoolbox_pose_node')
        self.subscription_amcl_pose = self.create_subscription(
            PoseWithCovarianceStamped,
            '/amcl_pose',
            self.pose_callback,
            10  
        )
        self.subscription_pose = self.create_subscription(
            PoseWithCovarianceStamped,
            '/pose',
            self.pose_callback,
            10  
        )

        self.publisher = self.create_publisher(PoseWithCovarianceStamped, '/robot_pose', 10)
        self.last_pose = None
        self.temp=None

  
    def pose_callback(self, msg):
        self.pose_data_x = msg.pose.pose.position.x
        self.pose_data_y = msg.pose.pose.position.y
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
            print("pose saved")
            file.close()

        self.last_pose = msg
        self.publish_pose()
        
    def publish_pose(self):
        if self.last_pose is not None:
            pose_msg = PoseWithCovarianceStamped()
            pose_msg.header.frame_id = 'map'
            now = self.get_clock().now()
            pose_msg.header.stamp = now.to_msg()
            pose_msg.pose = self.last_pose.pose
            self.publisher.publish(pose_msg)
            
def main(args=None):
    rclpy.init(args=args)
    node = MyNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
