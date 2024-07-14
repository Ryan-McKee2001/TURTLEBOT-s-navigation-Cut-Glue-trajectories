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

def main():
    '''This is a simple example of how to use the BasicNavigator class to send a goal pose to the navigation stack.
    This will be used for testing my different planner algorithms for time taken to reach the goal poses.'''
    rclpy.init()

    navigator = BasicNavigatorWithPose()

    # Spin until initial_pose is not None
    while navigator.get_pose() is None:
        rclpy.spin_once(navigator)

    initial_pose_with_covariance = navigator.get_pose()
    # print (str(initial_pose.pose.pose.position.x))
    initial_pose = PoseStamped()
    initial_pose.header = initial_pose_with_covariance.header
    initial_pose.pose = initial_pose_with_covariance.pose.pose

    navigator.setInitialPose(initial_pose)
    print ("Initial pose Received, Starting tests........ " )

    goal_poses = { 'goal1': {"x": 2.3, "y": -1.34, "w": 0.99, 'time': 0.0},
                    "goal2": {"x": 1.387, "y": 1.945, "w": 0.879, 'time': 0.0},
                     "goal3": {"x": 2.518, "y": 1.123, "w": 0.73, 'time': 0.0},
                      "goal4": {"x": 1.03655, "y": -1.010, "w": 0.062, 'time': 0.0},
                       "goal5": {"x": 3.796, "y": -0.219, "w": 0.193, 'time': 0.0},
    }

    navigator.setInitialPose(initial_pose)

    navigator.waitUntilNav2Active()
    for goal in goal_poses:
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = 'map'
        goal_pose.header.stamp = navigator.get_clock().now().to_msg()
        goal_pose.pose.position.x = goal_poses[goal]["x"]
        goal_pose.pose.position.y = goal_poses[goal]["y"]
        goal_pose.pose.orientation.w = goal_poses[goal]["w"]
        navigator.goToPose(goal_pose)

        i = 0
        while not navigator.isTaskComplete():

            # Do something with the feedback
            i = i + 1
            feedback = navigator.getFeedback()
            if feedback and i % 5 == 0:
                print(
                    'Estimated time of arrival: '
                    + '{0:.0f}'.format(
                        Duration.from_msg(feedback.estimated_time_remaining).nanoseconds
                        / 1e9
                    )
                    + ' seconds.'
                )

                # Some navigation timeout to demo cancellation
                if Duration.from_msg(feedback.navigation_time) > Duration(seconds=600.0):
                    navigator.cancelTask()

                # Some navigation request change to demo preemption
                if Duration.from_msg(feedback.navigation_time) > Duration(seconds=18.0):
                    goal_pose.pose.position.x = -3.0
                    navigator.goToPose(goal_pose)

        # Do something depending on the return code
        result = navigator.getResult()
        if result == TaskResult.SUCCEEDED:
            goal_poses[goal]["time"] = Duration.from_msg(feedback.navigation_time).nanoseconds / 1e9
            print('Goal succeeded! Time taken: ' + '{0:.0f}'.format(Duration.from_msg(feedback.navigation_time).nanoseconds / 1e9) + ' seconds.')
        elif result == TaskResult.CANCELED:
            print('Goal was canceled!')
        elif result == TaskResult.FAILED:
            print('Goal failed!')
        else:
            print('Goal has an invalid return status!')

    navigator.destroy_node()
    print ("Tests completed........ ")
    print("Goal times: ", goal_poses)
    exit(0)


if __name__ == '__main__':
    main()