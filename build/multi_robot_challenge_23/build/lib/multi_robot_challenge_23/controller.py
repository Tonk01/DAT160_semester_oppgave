import rclpy
from rclpy.node import Node

from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from rclpy.qos import qos_profile_sensor_data

# Most of the code is similar to previous oblig's we've been given. 
class TurtlebotController(Node): 
    def __init__(self):
        super().__init__("turtlebot_controller")

        self.stop_dist = 1.5

        self.scan_sub = self.create_subscription(
            LaserScan,
            '/tb3_0/scan',
            self.clbk_scan,
            qos_profile_sensor_data,
        )

        self.cmd_pub = self.create_publisher(Twist, '/tb3_0/cmd_vel', 10)

        self.get_logger().info("Controller started for tb3_0")

    # callback function for when a new LaserScan message arrives. 
    def clbk_scan(self, msg: LaserScan):
        twist = Twist()

        front_index = len(msg.ranges) // 2
        front_dist = msg.ranges[front_index]

        self.get_logger().info(f"Front dist :${front_dist:.2f} m")

        if front_dist <= self.stop_dist:
            twist.linear.x = 0.0
        else:
            twist.linear.x = 0.2

        self.cmd_pub.publish(twist)
    
def main(args=None):
    rclpy.init(args=args)

    controller = TurtlebotController()

    rclpy.spin(controller)

    controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
