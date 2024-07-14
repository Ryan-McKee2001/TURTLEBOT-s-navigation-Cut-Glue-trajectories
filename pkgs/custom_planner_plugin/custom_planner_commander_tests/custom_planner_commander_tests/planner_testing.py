#!/usr/bin/env python3
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
import rclpy.subscription
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy
# https://github.com/ros-planning/navigation2/blob/main/nav2_simple_commander/nav2_simple_commander/example_nav_to_pose.py

class BasicNavigatorWithPose(BasicNavigator):
    def __init__(self):
        super().__init__()
        qos = QoSProfile(depth=10, reliability=QoSReliabilityPolicy.RELIABLE, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        self.subscription = self.create_subscription(
            PoseWithCovarianceStamped,
            "/amcl_pose",
            self.pose_callback,
            qos
        )
        self.subscription  # prevent unused variable warning
        self.initial_pose = None

    def pose_callback(self, msg):
        print("pose_callback called")  # Debugging print
        print("Received initial pose: ", msg)
        self.initial_pose = msg  # store the pose
        # self.subscription.destroy()  # Temporarily commented out

    def destroy_node(self):
        self.subscription.destroy()
        super().destroy_node()

    def get_pose(self):
        return self.initial_pose

class PlannerTestingNode(Node):
    def __init__(self):
        super().__init__('planner_testing_node')

        self.navigator = BasicNavigatorWithPose()

        # Spin until initial_pose is not None
        while self.navigator.get_pose() is None:
            rclpy.spin_once(self.navigator)

        initial_pose_with_covariance = self.navigator.get_pose()
        initial_pose = PoseStamped()
        initial_pose.header = initial_pose_with_covariance.header
        initial_pose.pose = initial_pose_with_covariance.pose.pose

        self.navigator.setInitialPose(initial_pose)
        self.get_logger().info("Initial pose Received, Starting tests........ ")

        self.goal_poses = { 'goal1': {"x": 1.504, "y": -1.400, "w": 0.999, 'time': 0.0},
                            "goal2": {"x": 3.826, "y": 0.831, "w": 0.77, 'time': 0.0},
                            "goal3": {"x": 2.162, "y": 2.274, "w": 0.267, 'time': 0.0},
                            "goal4": {"x": 0.147, "y": 0.891, "w": 0.65, 'time': 0.0},
                            "goal5": {"x": 2.599, "y": 0.175, "w": 0.572, 'time': 0.0},
                            "goal6": {"x": 0.447, "y": 1.907, "w": 0.653, 'time': 0.0},
        }

        self.navigator.setInitialPose(initial_pose)

        self.navigator.waitUntilNav2Active()
        for goal in self.goal_poses:
            self.navigate_to_goal(goal)

        self.navigator.destroy_node()
        self.get_logger().info("Tests completed........ ")
        self.get_logger().info("Goal times: " + str(self.goal_poses))
        exit(0)

    def navigate_to_goal(self, goal):
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = 'map'
        goal_pose.header.stamp = self.navigator.get_clock().now().to_msg()
        goal_pose.pose.position.x = self.goal_poses[goal]["x"]
        goal_pose.pose.position.y = self.goal_poses[goal]["y"]
        goal_pose.pose.orientation.w = self.goal_poses[goal]["w"]
        self.navigator.goToPose(goal_pose)

        i = 0
        while not self.navigator.isTaskComplete():

            # Do something with the feedback
            i = i + 1
            feedback = self.navigator.getFeedback()
            if feedback and i % 5 == 0:
                self.get_logger().info(
                    'Estimated time of arrival: '
                    + '{0:.0f}'.format(
                        Duration.from_msg(feedback.estimated_time_remaining).nanoseconds
                        / 1e9
                    )
                    + ' seconds.'
                )

                # Some navigation timeout to demo cancellation
                if Duration.from_msg(feedback.navigation_time) > Duration(seconds=600.0):
                    self.navigator.cancelTask()

                # Some navigation request change to demo preemption
                if Duration.from_msg(feedback.navigation_time) > Duration(seconds=18.0):
                    goal_pose.pose.position.x = -3.0
                    self.navigator.goToPose(goal_pose)

        # Do something depending on the return code
        result = self.navigator.getResult()
        if result == TaskResult.SUCCEEDED:
            self.goal_poses[goal]["time"] = Duration.from_msg(feedback.navigation_time).nanoseconds / 1e9
            self.get_logger().info('Goal succeeded! Time taken: ' + '{0:.0f}'.format(Duration.from_msg(feedback.navigation_time).nanoseconds / 1e9) + ' seconds.')
        elif result == TaskResult.CANCELED:
            self.get_logger().info('Goal was canceled!')
        elif result == TaskResult.FAILED:
            self.get_logger().info('Goal failed!')
        else:
            self.get_logger().info('Goal has an invalid return status!')

def main(args=None):
    rclpy.init(args=args)

    node = PlannerTestingNode()

    rclpy.spin(node)

    rclpy.shutdown()


if __name__ == '__main__':
    main()