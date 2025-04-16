import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import numpy as np


class ObstacleAvoider(Node):
    '''
    Simple ROS2 node that keeps a Turtlebot3 Burger robot moving forward while avoiding obstacles.
    Made to be used for simulations with Gazebo (Turtlebot3 standard World)
    '''
    def __init__(self):
        super().__init__("obstacle_avoider")

        self.cruising_linear_velocity = 0.22 # max velocity for turtlebot3 burger (arbitrary)
        self.cruising_angular_velocity = np.pi / 2 # so that it makes a 90deg turn in 1 second (arbitrary)
        
        self.stop_dist_threshold = 0.3 # distance threshold for when to stop moving forward (arbitrary, but
        # below max linear velocity / scan refresh rate to avoid collisions)
        self.cruising_turn_dist_threshold = 0.6 # distance threshold for when to start turning (arbitrary)

        self.laserscan_sub = self.create_subscription(LaserScan, "scan", self.callback, 10)
        self.velocity_pub = self.create_publisher(Twist, "cmd_vel", 10)
        self.didnt_move_counter = 0 # used in case gets stuck

    def get_cmd_vel_msg(self, linear_velocity, angular_velocity) -> Twist:
        '''
        Create and return velocity command message from desired linear velocity (m/s) and angular velocity (rad/s)
        '''
        to_publish = Twist()
        to_publish.linear.x = float(linear_velocity) # float required by geometry_msgs.msg
        to_publish.angular.z = float(angular_velocity)
        return to_publish

    def callback(self, msg:Twist):
        '''
        Callback function triggered when laser scan data are received. 
        Determines whether to keep going forward and whether to turn based on laser scan distance measurements from 3 angles :
        30 degrees to the right, 20 degrees to the left, and right in front.
        30 is arbitrary, could very well be changed (bigger probably, smaller would mean the robot could graze on walls)
        '''
        
        is_stuck = True

        # get distance from 3 points : 30 deg left, in front, and 30 deg right. 
        scan_data = np.array(msg.ranges)
        dist_30deg_left = scan_data[29]
        dist_front = scan_data[0]
        dist_30deg_right = scan_data[330]
        measured_distances = (dist_30deg_left, dist_front, dist_30deg_right) 

        min_measured_distance = min(measured_distances) # find which one is closest

        if min_measured_distance <= self.stop_dist_threshold:
            new_linear_velocity = 0 # obstacle too close to continue cruising. stop
            self.didnt_move_counter += 1
        else :
            new_linear_velocity = self.cruising_linear_velocity # continue moving forward
            self.didnt_move_counter = 0
        
        if self.didnt_move_counter > 20:
            # simple get unstuck branch
            # if coudnt move for 20 callbacks : keep turning in the same direction (right) until able to move
            new_angular_velocity = - self.cruising_angular_velocity
        elif min_measured_distance <= self.cruising_turn_dist_threshold: 
            # obstacle is close : turning to avoid
            if min_measured_distance is dist_30deg_right:
                new_angular_velocity = self.cruising_angular_velocity # closest point is to the right. turn left
            elif min_measured_distance is dist_30deg_left:
                new_angular_velocity = - self.cruising_angular_velocity # closest point is to the left. turn right
            else: 
                # closest point is right in front. 2nd closest point will determine in which direction to turn.
                if min(dist_30deg_left, dist_30deg_right) is dist_30deg_right:
                    new_angular_velocity = self.cruising_angular_velocity # 2nd closest point is to the right. turn left
                else:
                    new_angular_velocity = - self.cruising_angular_velocity # 2nd closest point is to the left. turn right
        else :
            # obstacle is deemed far away :no need to turn
            new_angular_velocity = 0

        # generate command message, then publish it
        command_msg = self.get_cmd_vel_msg(linear_velocity=new_linear_velocity, angular_velocity=new_angular_velocity)
        self.velocity_pub.publish(command_msg)


def main(args=None):
    rclpy.init(args=args)

    obstacle_avoider_node = ObstacleAvoider()
    rclpy.spin(obstacle_avoider_node)

    rclpy.shutdown()

if __name__ == "__main__":
    main()
