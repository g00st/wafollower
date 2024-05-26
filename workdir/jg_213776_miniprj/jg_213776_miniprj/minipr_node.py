import rclpy
from rclpy.node import Node
from enum import Enum
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from miniprj_action_interfaces.action import Start
from rclpy.action import ActionServer
import numpy as np

class miniprj(Node):

    # ... other methods ...

    def laser_callback(self, msg):
        # Replace 0s with infinity
        msg.ranges = np.where(np.array(msg.ranges)==0, float('inf'), msg.ranges)
        self.laser_msg_ = msg



class RobotState(Enum):
    IDLE = 1
    DRIVE_TO_WALL = 2
    ROTATE = 3
    FOLLOW_WALL = 4

class miniprj(Node):

    def __init__(self):
        super().__init__('minipr_node')
        self.laser_msg_ = None
        self.state_ = RobotState.IDLE
        self.vel_publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        self.laser_subscriber_ = self.create_subscription(LaserScan, 'scan', self.laser_callback, 10)
        self.action_server_ = ActionServer(self, Start, 'start', self.Action_callback)

        self.target_distance_ = 0.3
        walleft = False

        if walleft:
            self.angelefront = 225
            self.angeleback = 315
            self.anglemiddle = 270

        else:
            self.angelefront = 45
            self.angeleback = 135
            self.anglemiddle = 90

    def laser_callback(self, msg):
        # Replace 0s with a large float value within the acceptable range
        msg.ranges = np.where(np.array(msg.ranges) == 0, float('inf'), msg.ranges).tolist()
        self.laser_msg_ = msg
    def Action_callback(self, goal_handle):
        self.get_logger().info('Received goal request')

        # --------------------- Check if the robot is already driving to the wall and subscriber/publisher are ready ---------------------

        if self.state_ == RobotState.IDLE:
            self.state_ = RobotState.DRIVE_TO_WALL
            self.get_logger().info('Driving to wall')
        else:
            self.get_logger().info('Already driving to wall')
            goal_handle.abort()
            return Start.Result()

        while self.laser_msg_ is None:
            self.get_logger().info('Waiting for laser scan')
            rclpy.spin_once(self)

        while self.vel_publisher_.get_subscription_count() == 0:
            self.get_logger().info('Waiting for subscriber')
            rclpy.spin_once(self)
       # --------------------- Main loop ---------------------
        while True:
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                self.get_logger().info('Goal was canceled')
                self.state_ = RobotState.IDLE
                return Start.Result("Goal was canceled")

            if self.state_ == RobotState.DRIVE_TO_WALL:
                self.drive_to_wall()
            elif self.state_ == RobotState.ROTATE:
                self.rotate()
            elif self.state_ == RobotState.FOLLOW_WALL:
                self.follow_wall()
            else:
                break
            rclpy.spin_once(self)

    def drive_to_wall(self):
            if self.laser_msg_.ranges[0] < self.target_distance_:
               self.state_ = RobotState.ROTATE
               self.get_logger().info('Rotating')
               msg = Twist()
               msg.linear.x = 0.0
               self.vel_publisher_.publish(msg)

            else:
                msg = Twist()
                msg.linear.x = 0.1
                self.vel_publisher_.publish(msg)

    def rotate(self):
            if round( self.laser_msg_.ranges[ self.angelefront],1 ) ==  round( self.laser_msg_.ranges[self.angeleback],1 ):
                self.state_ = RobotState.FOLLOW_WALL
                self.get_logger().info('Following wall')
                msg = Twist()
                msg.angular.z = 0.0
                self.vel_publisher_.publish(msg)
            else:
                msg = Twist()
                msg.angular.z = -0.2
                self.vel_publisher_.publish(msg)

    def follow_wall(self):
            msg = Twist()
            msg.linear.x = 0.1


            if self.laser_msg_.ranges[ self.anglemiddle  ] < self.target_distance_:
                delta = 0.2
            else:
                delta = -0.2

            if self.laser_msg_.ranges[ self.angelefront] > self.laser_msg_.ranges[self.angeleback]:
                msg.angular.z = 0.5 + delta
            elif self.laser_msg_.ranges[self.angelefront] < self.laser_msg_.ranges[self.angeleback]:
                msg.angular.z = -0.5 + delta
            else:  # self.laser_msg_.ranges[225] == self.laser_msg_.ranges[315]
                msg.angular.z = 0.0 + delta

            self.vel_publisher_.publish(msg)






def main():
    print('Hi from jg_213776_miniprj.')
    rclpy.init()
    my_node = miniprj()

    rclpy.spin(my_node)
def laser_callback(self, msg):
    # Replace 0s with a large float value within the acceptable range
    msg.ranges = np.where(np.array(msg.ranges) == 0, 1e+308, msg.ranges)
    self.laser_msg_ = msg

if __name__ == '__main__':
    main()
