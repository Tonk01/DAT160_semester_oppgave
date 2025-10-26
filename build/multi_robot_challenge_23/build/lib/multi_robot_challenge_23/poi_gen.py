
import rclpy
import math

from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Point
from std_msgs.msg import Bool
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

class PoiGen(Node):
    def __init__(self):
        super().__init__('poi_gen')

        self.declare_parameter('spacing', 5.0)          # meters between waypoints
        self.declare_parameter('free_threshold', 50) 
        self.declare_parameter('skip_unkn', True)       # skips -1 (unknown cells)

        self.spacing = float(self.get_parameter('spacing').value)
        self.free_threshold = int(self.get_parameter('free_threshold').value)
        self.skip_unkn = bool(self.get_parameter('skip_unkn').value)

        map_qos = QoSProfile(
            depth = 1, 
            reliability = ReliabilityPolicy.RELIABLE,
            durability = DurabilityPolicy.TRANSIENT_LOCAL
        )

        # topics (pubs & subs)
        self.goal_pub = self.create_publisher(Point, 'bug2/next_goal', 10)

        self.reached_sub = self.create_subscription(Bool, 'bug2/goal_reached', self.on_reached, 10)
        self.map_sub = self.create_subscription(OccupancyGrid, '/map', self.on_map, map_qos)

        self.grid = None
        self.seq = []
        self.idx = 0
        self.waiting_ack = False

        # period tick to re-send current goal
        self.create_timer(0.5, self.tick)

    # map callback, builds all waypoints once the map is received
    def on_map(self, msg: OccupancyGrid):
        self.grid = msg
        self.seq = self.make_waypoints(msg)
        self.get_logger().info(f"[poi_gen] map received: {msg.info.width}x{msg.info.height} @ {msg.info.resolution:.3f} m/px; waypoints={len(self.seq)}; spacing={self.spacing:.2f}")

        self.idx = 0
        self.waiting_ack = False

    # callback when robot reports it has reached the goal
    def on_reached(self, msg: Bool):
        if msg.data and self.idx < len(self.seq):
            self.get_logger().info(f"[poi_gen] waypoint {self.idx+1} reached; advancing")
            self.idx += 1
            self.waiting_ack = False

    # periodic tick, sending one goal at a time
    def tick(self):
        if self.grid is None:
            return
            
        if self.idx >= len(self.seq):
            return
            
        if self.waiting_ack:
            return
            
        p = self.seq[self.idx]
        self.get_logger().info(f"[poi_gen] sending waypoint {self.idx+1}/{len(self.seq)} -> (x={p.x:.2f}, y={p.y:.2f}) in odom")

        self.goal_pub.publish(p)
        self.waiting_ack = True
    
    # generates waypoints from the occupancy grid
    def make_waypoints(self, m: OccupancyGrid):
        res = m.info.resolution
        width = m.info.width
        height = m.info.height
        ox = m.info.origin.position.x
        oy = m.info.origin.position.y
        data = m.data

        # stride in cells based on spacing
        step = max(1, int(math.floor(self.spacing / res)))
        waypoints = []

        # lawnmover algorithm for the map, choosing only free cells
        for r in range(0, height, step):
            c_range = range(0, width, step) if (r // step) % 2 == 0 else range(width - 1, -1, -step)
            for c in c_range:
                i = r * width + c

                if i >= len(data):
                    continue

                v = data[i]

                if v == -1 and self.skip_unkn:
                    continue
                if v >= self.free_threshold:
                    continue

                x = ox + (c + 0.5) * res
                y = oy + (r  + 0.5) * res

                pt = Point(x = x, y = y, z = 0.0)
                waypoints.append(pt)

        return waypoints

def main(args = None):
    rclpy.init(args = args)
    node = PoiGen()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
