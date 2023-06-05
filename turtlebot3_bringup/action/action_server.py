#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from turtlebot3_bringup.action import DriveGoal
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator

class DriveGoal_server(Node):

    def __init__(self):
        super().__init__('drive_goal_server')
        self.action_server = ActionServer(
            self,
            DriveGoal,  
            'drive_goal',
            self.execute_callback
        )


    def execute_callback(self, goal_handle):
        feedback_msg = DriveGoal.Feedback()
        result_msg = DriveGoal.Result()

        goal_pose = goal_handle.request.goal_pose

        print("Pose x is: ", goal_pose.pose.position.x)
        print("Pose y is: ", goal_pose.pose.position.y)

        self.nav = BasicNavigator()
        init_pose = PoseStamped()

        self.nav.setInitialPose(init_pose)
        self.nav.waitUntilNav2Active() # if autostarted, else use lifecycleStartup()

        self.nav.goToPose(goal_pose)
        while not self.nav.isTaskComplete():
          feedback = self.nav.getFeedback()
          feedback_msg.current_pose = feedback.current_pose
          goal_handle.publish_feedback(feedback_msg)
          print("current_pose :",feedback.current_pose)
          print("navigation_time :",feedback.navigation_time.sec)
          print("estimated_time_remaining :",feedback.estimated_time_remaining.sec)
          print("number_of_recoveries :",feedback.number_of_recoveries)
          print("distance_remaining :",feedback.distance_remaining)

        #   if feedback.navigation_time.sec > 60:
        #     print("in cancel if")
        #     self.nav.cancelTask()

        result = self.nav.getResult()        

        print("result :", result)

        if result == "TaskResult.SUCCEEDED":
            result_msg = True
            print('Goal succeeded!')
        elif result == "TaskResult.CANCELED":
            print('Goal was canceled!')
            result_msg=False
        elif result == "TaskResult.FAILED":
            result_msg=False
            print('Goal failed!')

        goal_handle.succeed()

        return result_msg


def main():

    rclpy.init()

    rclpy.spin(DriveGoal_server())
    DriveGoal_server().destroy_node()

    rclpy.shutdown()

if __name__ == '__main__':
    main()
