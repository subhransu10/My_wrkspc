import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan


class ObstacleAvoider(Node):
    def __init__(self):
        super().__init__('obstacle_avoider')
        self.cmd_pub = self.create_publisher(Twist, '/demo/cmd_vel', 10)
        self.scan_sub = self.create_subscription(LaserScan, '/demo/scan', self.scan_callback, 10)
        self.get_logger().info('ObstacleAvoider node started.')

    def scan_callback(self, msg):
        front_angles = msg.ranges[len(msg.ranges)//2 - 10 : len(msg.ranges)//2 + 10]
        min_distance = min(front_angles)

        twist = Twist()

        # Avoid if obstacle is close
        if min_distance < 0.6:
            self.get_logger().info(f'Obstacle ahead: {min_distance:.2f}m → Turning')
            twist.angular.z = 0.6  # Turn away
        else:
            twist.linear.x = 0.2   # Move forward

        self.cmd_pub.publish(twist)


def main(args=None):
    rclpy.init(args=args)
    node = ObstacleAvoider()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
