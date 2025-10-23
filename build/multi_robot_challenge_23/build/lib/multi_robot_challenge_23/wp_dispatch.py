
import rclpy 
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from collections import deque

from std_msgs.msg import Bool
from geometry_msgs.msg import Point

class WaypointDispatcher(Node):
    def __init__(self):
        super().__init__('wp_dispatcher')

        self.robots = ['tb3_0', 'tb3_1']
        
        # FIFO
        self.queue = deque()
        self.available = {ns: False for ns in self.robots}

        latched_qos = QoSProfile(depth = 1, reliability = ReliabilityPolicy.RELIABLE, durability = DurabilityPolicy.TRANSIENT_LOCAL)

        # global I/O's 
        self.sub_wp = self.create_subscription(Point, '/global/waypoints', self.on_waypoint, 10)
        self.pub_global_reached = self.create_publisher(Bool, '/global/goal_reached', 10)


        self.robot_pub_goal = { ns: self.create_publisher(Point, f'/{ns}/bug2/next_goal', 10) for ns in self.robots}
        self.robot_sub_avail = { ns: self.create_subscription(Bool, f'/{ns}/bug2/avilable', lambda msg, ns = ns: self.on_available(ns, msg), latched_qos) for ns in self.robots}
        self.robot_sub_reached = { ns: self.create_subscription(Bool, f'/{ns}/bug2/goal_reached', lambda msg, ns = ns: self.on_goal_reached(ns, msg), 10) for ns in self.robots}

        self.create_timer(0.5, self.try_dispatch)
        self.get_logger().info("wp_dispatcher ready: FIFO")

    def on_waypoint(self, msg: Point):
        self.queue.append(msg)

    def on_available(self, ns: str, msg: Bool):
        self.available[ns] = bool(msg.data)

    def on_goal_reached(self, ns: str, msg: Bool):
        if not msg.data:
            return
        
        # robot has reached the goal, becomes available
        self.available[ns] = True
        self.pub_global_reached.publish(Bool(data = True))

    def try_dispatch(self):
        if not self.queue:
            return
        
        target_ns = next((ns for ns in self.robots if self.available.get(ns, False)), None)

        if target_ns is None:
            return

        # assigning next waypoint
        wp = self.queue.popleft()
        self.robot_pub_goal[target_ns].publish(wp)

        self.available[target_ns] = False
        self.get_logger().info(f"assigned waypoint ({wp.x:.2f}, {wp.y:.2f}) to {target_ns}")

def main(args = None):
    rclpy.init(args = args)
    node = WaypointDispatcher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
