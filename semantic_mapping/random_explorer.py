import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import random
import math

class RandomExplorer(Node):
    def __init__(self):
        super().__init__('random_explorer')
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.sub = self.create_subscription(
            LaserScan, 'scan', self.scan_callback, 10
        )
        self.state = 'forward'
        self.rotate_count = 0
        self.rotate_target = 0
        self.reverse_count = 0
        self.obstacle_ahead = False
        self.obstacle_left = False
        self.obstacle_right = False
        self.close_obstacle = False

        self.timer = self.create_timer(0.1, self.control_loop)
        self.stuck_timer = self.create_timer(5.0, self.stuck_check)
        self.last_state_change = 0
        self.control_cycles = 0

        self.get_logger().info('Random explorer started')

    #use lidar to detect when an obstacle is nearby
    def scan_callback(self, msg):
        ranges = msg.ranges
        n = len(ranges)

        def min_range(indices):
            vals = [ranges[i] for i in indices if 0 <= i < n and math.isfinite(ranges[i])]
            return min(vals) if vals else float('inf')

        fwd = list(range(0, n//12)) + list(range(11*n//12, n))
        fwd_wide = list(range(0, n//8)) + list(range(7*n//8, n))
        left = list(range(n//12, 3*n//12))
        right = list(range(9*n//12, 11*n//12))

        fwd_min = min_range(fwd)
        fwd_wide_min = min_range(fwd_wide)

        self.close_obstacle = fwd_min < 0.25
        self.obstacle_ahead = fwd_wide_min < 0.55
        self.obstacle_left = min_range(left) < 0.35
        self.obstacle_right = min_range(right) < 0.35


    def stuck_check(self):
        if self.state == 'rotating' and self.rotate_count > 60:
            self.get_logger().warn('stuck started reversing')
            self.state = 'reversing'
            self.reverse_count = 0

    #turning logic - forward is possile, left if obstacle on right, right if obstacle on left
    def control_loop(self):
        twist = Twist()

        if self.state == 'forward':
            if self.close_obstacle:
                self.state = 'reversing'
                self.reverse_count = 0
            elif self.obstacle_ahead:
                self.state = 'rotating'
                self.rotate_count = 0
                if self.obstacle_left and not self.obstacle_right:
                    self.rotate_target = random.randint(8, 15)
                    twist.angular.z = -0.4
                elif self.obstacle_right and not self.obstacle_left:
                    self.rotate_target = random.randint(8, 15)
                    twist.angular.z = 0.4
                else:
                    self.rotate_target = random.randint(10, 25)
                    twist.angular.z = 0.4 * random.choice([-1, 1])
            else:
                twist.linear.x = 0.10

        elif self.state == 'rotating':
            self.rotate_count += 1
            twist.angular.z = 0.4
            if self.rotate_count >= self.rotate_target and not self.obstacle_ahead:
                self.state = 'forward'
                self.rotate_count = 0

        elif self.state == 'reversing':
            self.reverse_count += 1
            twist.linear.x = -0.10
            if self.reverse_count >= 20:
                self.state = 'rotating'
                self.rotate_count = 0
                self.rotate_target = random.randint(15, 35)
        
        self.pub.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    node = RandomExplorer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
