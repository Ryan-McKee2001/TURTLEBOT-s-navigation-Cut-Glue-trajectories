import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from enum import Enum
import time
import math

class TurnDirection(Enum):
    CLOCKWISE = 'clockwise'
    ANTI_CLOCKWISE = 'anti_clockwise'
    STRAIGHT = 'straight'

class RobotControl(Node):
    def __init__(self):
        super().__init__('turtlebot3_robot_control')
        self.get_logger().info('Initializing turtlebot3 robot control /cmd_vel publisher...')
        self._rc_publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.get_logger().info('Successfully initialized turtlebot3 robot controller cmd_vel publisher node.')

    def send_rc_msg(self, linear_speed: float, turn_speed: float, turn_direction: TurnDirection):
        twist = Twist()
        twist.linear.x = linear_speed
        twist.angular.z = 0.0

        if turn_direction != TurnDirection.STRAIGHT:
            twist.angular.z = -turn_speed if turn_direction == TurnDirection.CLOCKWISE else turn_speed

        self._rc_publisher.publish(twist)

class WallFollower(Node):
    def __init__(self):
        super().__init__("turtlebot3_laser_scan_subscriber")
        self.get_logger().info('Initializing /cmd_vel publisher...')
        self._cmd_vel_publisher = RobotControl()
        self.get_logger().info('Initializing /scan subscriber...')
        self._laser_scan_subscriber = self.create_subscription(
            LaserScan,
            "/scan",
            self.laser_callback,
            qos_profile=rclpy.qos.qos_profile_sensor_data
        )
        self.get_logger().info('Successfully subscribed to /scan...')

    def laser_callback(self, msg: LaserScan):
        time.sleep(0.3) # seconds
        self._message = msg # Store the message for later access
        self.ray_range_right = self._get_ray_range_at_degree(163)
        if not math.isnan(self.ray_range_right):
            ray_range_front = self._get_ray_range_at_degree(0)
            linear_speed = 0.1 # Increase linear speed for faster movement
            turn_speed = 0.1

            if ray_range_front >= 1.0:
                if self.ray_range_right > 2.0:
                    self._cmd_vel_publisher.send_rc_msg(linear_speed=linear_speed, turn_speed=turn_speed, turn_direction=TurnDirection.CLOCKWISE)
                    self.get_logger().info('Wall too far away on right, Turning clockwise')
                elif self.ray_range_right < 1.5:
                    self._cmd_vel_publisher.send_rc_msg(linear_speed=linear_speed, turn_speed=turn_speed, turn_direction=TurnDirection.ANTI_CLOCKWISE)
                    self.get_logger().info('Wall too close on right, Turning ANTI CLOCKWISE')
                else:
                    self._cmd_vel_publisher.send_rc_msg(linear_speed=linear_speed, turn_speed=0.0, turn_direction=TurnDirection.STRAIGHT)
                    self.get_logger().info('Going straight')
            else:
                self._cmd_vel_publisher.send_rc_msg(linear_speed=0.0, turn_speed=0.5, turn_direction=TurnDirection.ANTI_CLOCKWISE)
                self.get_logger().info('Wall in front too close turn 90 degrees anti clockwise')

    def _get_ray_range_at_degree(self, degree: int):
        return self._message.ranges[degree]

def main():
    rclpy.init()
    try:
        wall_follower_node = WallFollower()
        rclpy.spin(wall_follower_node)
    except Exception as e:
        print('Exception occurred:', e)
    finally:
        if 'wall_follower_node' in locals():
            wall_follower_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()