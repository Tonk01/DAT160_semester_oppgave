import rclpy
import math
from rclpy.node import Node

from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from rclpy.qos import qos_profile_sensor_data

# Most of the code is similar to previous oblig's we've been given. 
class TurtlebotController(Node): 
    def __init__(self):
        super().__init__("turtlebot_controller")
        
        self.scan_sub = self.create_subscription(
            LaserScan,
            '/tb3_1/scan',
            self.clbk_scan,
            qos_profile_sensor_data,
        )

        # Publisher for tb3_0 
        self.cmd_pub = self.create_publisher(
            Twist, 
            '/tb3_1/cmd_vel', 
            10
        )

        self.left_min = float('inf')
        self.right_min = float('inf')
        self.front_min = float ('inf')

        self.timer = self.create_timer(0.1, self.clbk_timer)
        self.get_logger().info("Controller started for tb3_1")

    # callback function for when a new LaserScan message arrives. 
    def clbk_scan(self, msg: LaserScan):

        valid_ranges = [r for r in msg.ranges if not math.isinf(r) and not math.isnan(r)]
        if not valid_ranges:
            return
        
        n = len(msg.ranges)
        
        #degree calculations provided by chatGPT. 
        front = msg.ranges[n - 20:] + msg.ranges[:21]       # -20...+20 degrees
        left = msg.ranges[n//4 : 3*n//4]                    # 90 - 270 degrees
        right = msg.ranges[3*n//4:] + msg.ranges[:n//4]     # 270 - 90 degrees

        def vmin(arr):
            vals = [r for r in arr if r > 0.0 and not math.isinf(r) and not math.isnan(r)]
            return min(vals) if vals else float('inf')
    
        self.front_min = vmin(front)
        self.left_min = vmin(left)
        self.right_min = vmin(right)

    def clbk_timer(self):
        vel_msg = Twist()
        detect_range = 0.5
        
        # turning logic.
        if self.front_min < detect_range:
            if self.left_min < self.right_min:
                vel_msg.angular.z = -0.5
            else:
                vel_msg.angular.z = 0.5
            vel_msg.linear.x = 0.0
        else:
            vel_msg.linear.x = 0.4
            vel_msg.angular.z = 0.0

        self.cmd_pub.publish(vel_msg)
    
def main(args=None):
    rclpy.init(args=args)

    node  = TurtlebotController()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
