
import rclpy
import numpy as np

from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from std_srvs.srv import SetBool


class WallFollower(Node):
    def __init__(self):
        super().__init__('wall_follower_hybrid')

        # topics
        self.scan_sub = self.create_subscription(LaserScan, 
            'scan', 
            self.clbk_scan, 
            10
            )
        
        self.cmd_pub  = self.create_publisher(Twist, 'cmd_vel', 10)

        self.enabled = False
        self.srv_switch = self.create_service(SetBool, 'wall_follower/switch', self.on_switch)

        # change any of these values below at your own risk. GL

        # tuning
        deg = np.deg2rad
        self.tol = deg(2)
        self.tol_go = deg(6)
        self.tol_stop = deg(10)
        self.tol_move = deg(3)

        # speeds
        self.k = 0.7
        self.max_w = 0.7
        self.max_w_move = 0.7
        self.v_fwd = 0.20

        # distances
        self.approach_stop = 0.5
        self.dist_safe = 0.25

        # heading
        self.alpha = 0.8

        # states
        self.mode = 'face'
        self.moving = False
        self.heading = None

        # timer (used for checking wall distance)
        self.recheck_timer = self.create_timer(40.0, self.recheck)
        
    # wraps pi (-pi, pi]
    @staticmethod
    def wrap(x):
        return (x + np.pi) % (2 * np.pi) - np.pi
    
    # switch to enable and disable wall follower, also starts wall follower by putting mode into 'face'
    def on_switch(self, request: SetBool.Request, response: SetBool.Response):
        self.enabled = bool(request.data)

        self.moving = False
        self.heading = None
        self.mode = 'face'
        self.cmd_pub.publish(Twist())

        response.success = True
        response.message = 'wall_follower enabled' if self.enabled else 'wall_follower disabled'
        return response
        
    # checks distance to wall and swaps to 'face' mode inorder to get closer. 
    # made to combat the big "swings" from going around corners.
    def recheck(self):
        cmd = Twist()

        if not self.enabled:
            return
    
        # checks if we're in parallel and have a stored front value to use. 
        if self.mode == 'parallel' and hasattr(self, "last_front"):
            f = self.last_front
            self.get_logger().info(f"Recheck: front_min={f:.2f}, approach_stop={self.approach_stop:.2f}")

            if np.isfinite(f) and f > self.approach_stop:
                self.get_logger().info("Recheck -> too far, switching to face")

                self.mode = 'face'
                self.heading = None
                self.moving = False
                self.cmd_pub.publish(cmd)
                self.get_logger().info("Recheck")

    # scan callback. 
    def clbk_scan(self, msg: LaserScan):
        cmd = Twist()

        if not self.enabled:
            self.cmd_pub.publish(Twist())
            return

        # simple check for finite values (useable range scans)
        r = np.array(msg.ranges, dtype = np.float32)
        r[~np.isfinite(r)] = np.inf

        if msg.range_min > 0.0:
            r = np.where(r < msg.range_min, np.inf, r)
            
        if np.isfinite(msg.range_max) and msg.range_max > 0.0:
            r = np.where(r > msg.range_max, np.inf, r)

        if not np.any(np.isfinite(r)):
            self.cmd_pub.publish(Twist())
            return

        n = r.size
        ang = msg.angle_min + np.arange(n, dtype = np.float32) * msg.angle_increment
        ang = np.arctan2(np.sin(ang), np.cos(ang))

        weight = r * (1.0 + 0.25 * np.abs(ang) / np.pi)                             # weighting to help decide a "winner" if two walls are to far
        idx = int(np.argmin(np.where(np.isfinite(weight), weight, np.inf)))
        target_angle = float(ang[idx])

        deg = np.deg2rad
        front = np.abs(ang - 0.0) <= deg(5)

        front_min = float(np.min(r[front])) if np.any(front) else float('inf')
        self.last_front = front_min

        # Turn towards nearest wall, then move to 'approach'
        if self.mode == 'face':
            err = self.wrap(target_angle)
            if abs(err) > self.tol:
                cmd.angular.z = float(np.clip(self.k * err, -self.max_w, self.max_w))
                cmd.linear.x = 0.0
            else:
                self.mode = 'approach'
                cmd.angular.z = 0.0
                cmd.linear.x = 0.0

        # 'approach' mode goes towards the nearest wall.
        elif self.mode == 'approach':
            err = self.wrap(target_angle)
            cmd.angular.z = float(np.clip(self.k * err, -self.max_w, self.max_w)) if abs(err) > self.tol else 0.0

            # stop when face (front) is close enough to the wall and switch to parallel mode 
            if front_min <= self.approach_stop:
        
                self.mode = 'parallel'
                self.heading = None
                cmd.angular.z = 0.0
                cmd.linear.x = 0.0

            elif abs(err) <= self.tol:
                cmd.linear.x = 0.20
            else:
                cmd.linear.x = 0.0
        

        # parallel section...
        else: 
            desired = self.wrap(target_angle + np.pi/ 2)

            # sets desired heading if no heading exists. Then calculate heading.
            if self.heading is None:
                self.heading = desired
            else:
                self.heading = self.wrap(self.heading + self.alpha * self.wrap(desired - self.heading))
                
                err = self.wrap(self.heading)

                can_go = (abs(err) <= self.tol_go) and (front_min > self.approach_stop)
                must_stop = (abs(err) > self.tol_stop) or (front_min <= self.dist_safe)

                # boolean blocks that allows for adjustments during movement. 
                if self.moving:
                    if must_stop:
                        self.moving = False
                
                else:
                    if can_go:
                        self.moving = True
                
                # continous fine adjustments and safety adjustments. (creates a little overhead while moving, but it works.)
                err_cmd = 0.0 if (self.moving and abs(err) < self.tol_move) else err

                max_w_now = self.max_w_move if self.moving else self.max_w
                cmd.angular.z = float(np.clip(self.k * err_cmd, -max_w_now, max_w_now))
                cmd.linear.x = self.v_fwd if self.moving else 0.0

                # collision protection, litteraly an emergency break :)
                if front_min <= self.dist_safe:
                    self.moving = False
                    self.mode = 'approach'
                    cmd.angular.z = 0.0
                    cmd.linear.x = 0.0
                
        self.cmd_pub.publish(cmd)

def main(args=None):
    rclpy.init(args=args)
    node = WallFollower()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

