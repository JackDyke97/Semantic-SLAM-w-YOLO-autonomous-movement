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
        left = list(range(n//12, 3*n//12))
        right = list(range(9*n//12, 11*n//12))

        self.obstacle_ahead = min_range(fwd) < 0.4
        self.obstacle_left = min_range(left) < 0.3
        self.obstacle_right = min_range(right) < 0.3


    def stuck_check(self):
        if self.state == 'rotating' and self.rotate_count > 60:
            self.get_logger().warn('stuck started reversing')
            self.state = 'reversing'
            self.reverse_count = 0

    #turning logic - forward is possile, left if obstacle on right, right if obstacle on left
    #otherwise cry
    def control_loop(self):
        twist = Twist()
        self.control_cycles += 1

        if self.state == 'forward':
            if self.obstacle_ahead:
                self.state = 'rotating'
                self.rotate_count = 0
                if self.obstacle_left and not self.obstacle_right:
                    self.rotate_target = random.randint(8, 15)
                    twist.angular.z = -0.5
                elif self.obstacle_right and not self.obstacle_left:
                    self.rotate_target = random.randint(8, 15)
                    twist.angular.z = 0.5
                else:
                    self.rotate_target = random.randint(10, 25)
                    twist.angular.z = 0.5 * random.choice([-1, 1])
            else:
                twist.linear.x = 0.15
        
        elif self.state == 'rotating':
            self.rotate_count += 1
            twist.angular.z = 0.5
            if self.rotate_count >= self.rotate_target and not self.obstacle_ahead:
                self.state = 'forward'
                self.rotate_count = 0

        elif self.state == 'reversing':
            self.reverse_count += 1
            twist.linear.x = -0.12
            if self.reverse_count >= 15:
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