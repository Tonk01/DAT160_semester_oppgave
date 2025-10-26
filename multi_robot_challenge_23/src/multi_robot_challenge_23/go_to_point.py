
import numpy as np
import rclpy
import math

from tf_transformations import euler_from_quaternion
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from bug2_interface.srv import GoToPoint as Gpsrv

# helper method wraping/normalizing angle to (-π, π]
def wrap_pi(x):
    return (x + math.pi) % (2.0 * math.pi) - math.pi

class GoToPoint(Node):
    def __init__(self):
        super().__init__('go_to_point')

        self.odom_sub = self.create_subscription(
            Odometry,
            'odom',
            self.clbk_odom,
            20
        )

        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.srv = self.create_service(Gpsrv, 'go_to_point/switch', self.srv_clbk)

        self.position = None
        self.yaw = None 
        self.goal = None          # positions

        self.yaw_k = 1.0               # heading 
        self.max_w = 0.6
        self.max_w_move = 0.3
        self.yaw_tol = math.radians(3)  # yaw deadband
        self.velo = 0.25                # forwarding speed
        self.stop_dist = 0.03           
        self.slow_radius = 0.6

        self.create_timer(0.05, self.control)

    # callback, triggering GoToPoint behavior
    def srv_clbk(self, req: Gpsrv.Request, resp: Gpsrv.Response):
        if req.move_switch:
            self.goal = (req.target_position.x, req.target_position.y)  # turn on and set goal
        else: 
            self.goal = None
            self.cmd_pub.publish(Twist())

        resp.success = True
        return resp
    
    # odom callback
    def clbk_odom(self, msg: Odometry):
        self.position = msg.pose.pose.position

        q = msg.pose.pose.orientation
        euler = euler_from_quaternion((q.x, q.y, q.z, q.w))
        self.yaw = wrap_pi(euler[2])


    # Calculates the angle & moves the robot. [main control loop]
    def control(self):
        cmd = Twist()

        if self.position is None or self.yaw is None:
            self.cmd_pub.publish(cmd)
            return
        
        if self.goal is None:
            return

        dx = self.goal[0] - self.position.x
        dy = self.goal[1] - self.position.y
        dist = math.hypot(dx, dy)               # vector goal

        if dist <= self.stop_dist:
            self.cmd_pub.publish(cmd)
            return

        # calculate target yaw
        target_yaw = math.atan2(dy, dx)
        yaw_err = wrap_pi(target_yaw - self.yaw)

        # movement segment. 
        if abs(yaw_err) > self.yaw_tol:
            cmd.angular.z = float(np.clip(self.yaw_k * yaw_err, -self.max_w, self.max_w))
            cmd.linear.x = 0.0
        else:
            cmd.angular.z = float(np.clip(self.yaw_k * yaw_err, -self.max_w_move, self.max_w_move))
            gain = min(1.0, max(dist / self.slow_radius, 0.15))
            cmd.linear.x = self.velo * gain
        
        self.cmd_pub.publish(cmd)

def main(args=None):
    rclpy.init(args=args)
    node = GoToPoint()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()