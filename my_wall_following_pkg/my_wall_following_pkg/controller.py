import math
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from rclpy.qos import qos_profile_sensor_data
import numpy as np

class Controller(Node):
    def __init__(self):
        super().__init__('Controller')

        # Subscriptions
        self.create_subscription(Float64MultiArray, '/demo/state_est', self.state_estimate_callback, 10)
        self.create_subscription(LaserScan, '/demo/laser/out', self.scan_callback, qos_profile=qos_profile_sensor_data)

        # Publisher
        self.publisher_ = self.create_publisher(Twist, '/demo/cmd_vel', 10)

        # Laser distances
        self.left_dist = 999.9
        self.leftfront_dist = 999.9
        self.front_dist = 999.9
        self.rightfront_dist = 999.9
        self.right_dist = 999.9

        # Parameters
        self.forward_speed = 0.15
        self.turning_speed_wf_fast = 2.0
        self.turning_speed_wf_slow = 0.1
        self.dist_thresh_wf = 0.75
        self.dist_too_close_to_wall = 0.4
        self.wall_following_state = "init"

    def avg_range(self, ranges, idx, window=5):
        window_values = [r for r in ranges[max(0, idx - window):min(len(ranges), idx + window)] if 0.05 < r < 10.0]
        return np.mean(window_values) if window_values else 10.0

    def scan_callback(self, msg):
        self.left_dist = self.avg_range(msg.ranges, 180)
        self.leftfront_dist = self.avg_range(msg.ranges, 135)
        self.front_dist = self.avg_range(msg.ranges, 90)
        self.rightfront_dist = self.avg_range(msg.ranges, 45)
        self.right_dist = self.avg_range(msg.ranges, 0)

    def state_estimate_callback(self, msg):
        self.follow_wall()

    def follow_wall(self):
        msg = Twist()
        d = self.dist_thresh_wf
        t = self.dist_too_close_to_wall

        front_clear = self.front_dist > d
        left_clear = self.leftfront_dist > d
        right_clear = self.rightfront_dist > d

        too_close_front = self.front_dist < t
        too_close_right = self.rightfront_dist < t
        too_close_left = self.leftfront_dist < t

        if too_close_front or too_close_right or too_close_left:
            self.wall_following_state = "EMERGENCY TURN"
            msg.linear.x = 0.0
            msg.angular.z = self.turning_speed_wf_fast

        elif not front_clear:
            self.wall_following_state = "Avoid Front Wall"
            msg.linear.x = 0.0
            msg.angular.z = self.turning_speed_wf_fast

        elif not right_clear and front_clear:
            self.wall_following_state = "Follow Wall"
            msg.linear.x = self.forward_speed
            msg.angular.z = 0.0

        elif front_clear and right_clear:
            self.wall_following_state = "Search for Wall"
            msg.linear.x = self.forward_speed
            msg.angular.z = -self.turning_speed_wf_slow

        else:
            self.wall_following_state = "Default Avoid"
            msg.angular.z = self.turning_speed_wf_fast

        self.get_logger().info(
            f"[{self.wall_following_state}] Front: {self.front_dist:.2f}, "
            f"R-Front: {self.rightfront_dist:.2f}, L-Front: {self.leftfront_dist:.2f}"
        )
        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    controller = Controller()
    rclpy.spin(controller)
    controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
