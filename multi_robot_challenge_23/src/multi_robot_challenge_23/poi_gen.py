
import rclpy
import math

from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Point
from std_msgs.msg import Bool
from collections import deque
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

class PoiGen(Node):
    def __init__(self):
        super().__init__('poi_gen')

        self.declare_parameter('spacing', 3.0)          # meters between waypoints
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

        qos_goal = QoSProfile(
            depth = 1,
            reliability = ReliabilityPolicy.RELIABLE,
            durability = DurabilityPolicy.TRANSIENT_LOCAL
        )

        # topics (pubs & subs)
        self.goal_pub = self.create_publisher(Point, 'bug2/next_goal', 10)

        self.goal_pub_0 = self.create_publisher(Point, '/tb3_0/bug2/next_goal', qos_goal)
        self.goal_pub_1 = self.create_publisher(Point, '/tb3_1/bug2/next_goal', qos_goal)

        self.reached_sub = self.create_subscription(Bool, 'bug2/goal_reached', self.on_reached, 10) # <-- unsued for this
        self.reached_sub_0 = self.create_subscription(Bool, '/tb3_0/bug2/goal_reached', self.on_reached_0, 10)
        self.reached_sub_1 = self.create_subscription(Bool, '/tb3_1/bug2/goal_reached', self.on_reached_1, 10)
        
        self.map_sub = self.create_subscription(OccupancyGrid, '/map', self.on_map, map_qos)

        self.grid = None
        self.seq = []
        self.idx = 0
        self.waiting_ack = False

        self.seq0 = []
        self.seq1 = []
        self.idx0 = 0
        self.idx1 = 0
        self.waiting_ack0 = False
        self.waiting_ack1 = False

        # period tick to re-send current goal
        self.create_timer(0.5, self.tick)

    # map callback, builds all waypoints once the map is received
    def on_map(self, msg: OccupancyGrid):
        self.grid = msg
        self.seq = self.make_waypoints(msg)
        self.get_logger().info(f"[poi_gen] map received: {msg.info.width}x{msg.info.height} @ {msg.info.resolution:.3f} m/px; waypoints={len(self.seq)}; spacing={self.spacing:.2f}")

        # split the generated waypoints, first half -> tb3_0, second half -> tb3_1
        half = len(self.seq) // 2
        self.seq0 = self.seq[:half]
        self.seq1 = self.seq[half:]

        self.idx = 0
        self.waiting_ack = False

        self.idx0 = 0
        self.idx1 = 0
        self.waiting_ack0 = False
        self.waiting_ack1 = False

        self.get_logger().info(f"[poi_gen] split:  tb3_0={len(self.seq0)} waypoints, tb3_1={len(self.seq1)} waypoints")

    # callback when robot reports it has reached the goal <-- unused for now. 
    def on_reached(self, msg: Bool):
        if msg.data and self.idx < len(self.seq):
            self.get_logger().info(f"[poi_gen] waypoint {self.idx+1} reached; advancing")
            self.idx += 1
            self.waiting_ack = False

    # tb3_0 ack
    def on_reached_0(self, msg: Bool):
        if msg.data and self.idx0 < len(self.seq0):
            self.get_logger().info(f"[poi_gen] tb3_0 waypoint {self.idx0+1} reached; advancing")
            self.idx0 += 1
            self.waiting_ack0 = False

    # tb3_1 ack
    def on_reached_1(self, msg: Bool):
        if msg.data and self.idx1 < len(self.seq1):
            self.get_logger().info(f"[poi_gen] tb3_1 waypoint {self.idx1+1} reached; advancing")
            self.idx1 += 1
            self.waiting_ack1 = False

    # periodic tick, sending one goal at a time
    def tick(self):
        if self.grid is None:
            return
            
        # tb3_0
        if self.idx0 < len(self.seq0) and not self.waiting_ack0:
            p0 = self.seq0[self.idx0]
            self.get_logger().info(f"[poi_gen] tb3_0 sending waypoint {self.idx0+1}/{len(self.seq0)} -> (x={p0.x:.2f}, y={p0.y:.2f}) in odom")
            self.goal_pub_0.publish(p0)
            self.waiting_ack0 = True

        # tb3_1
        if self.idx1 < len(self.seq1) and not self.waiting_ack1:
            p1 = self.seq1[self.idx1]
            self.get_logger().info(f"[poi_gen] tb3_1 sending waypoint {self.idx1+1}/{len(self.seq1)} -> (x={p1.x:.2f}, y={p1.y:.2f}) in odom")
            self.goal_pub_1.publish(p1)
            self.waiting_ack1 = True


    # generates waypoints from the occupancy grid
    def make_waypoints(self, m: OccupancyGrid):
        res = m.info.resolution
        width = m.info.width
        height = m.info.height
        ox = m.info.origin.position.x
        oy = m.info.origin.position.y
        data = m.data

        # Paramaters
        free_thre = self.free_threshold
        skip_unkn = self.skip_unkn
        step = max(1, int(math.floor(self.spacing / res)))
        inflate_m = getattr(self, "inflate_margin_m", 0.10)
        inflate_px = max(0, int(round(inflate_m / res)))
        
        # small helpers
        def idx(r, c): 
            return r * width + c
        
        def is_free(v):
            if v == -1:
                return not skip_unkn
            return v < free_thre
        
        # Build free mask
        free = [False] * (width * height)
        for r in range(height):
            base = r * width
            for c in range(width):
                v = data[base + c]
                free[base + c] = is_free(v)

        if inflate_px > 0:
            near_occ = [False] * (width * height)
            
            for r in range(height):
                for c in range(width):
                    if not free[idx(r, c)]:

                        # mark neighbors within inflate_px as near occupied
                        r0 = max(0, r - inflate_px)
                        r1 = min(height - 1, r + inflate_px)

                        c0 = max(0, c - inflate_px)
                        c1 = min(width - 1, c + inflate_px)

                        for rr in range(r0, r1 + 1):
                            base = rr * width

                            for cc in range(c0, c1 + 1):
                                near_occ[base + cc] = True

            for i in range(width * height):
                if free[i] and near_occ[i]:
                    free[i] = False

        outside = [False] * (width * height)
        q = deque()

        def push_if_border_free(r, c):
            if 0 <= r < height and 0 <= c < width:

                i = idx(r, c)
                if free[i] and not outside[i]:
                    outside[i] = True
                    q.append((r, c))

        # enque all border free cells
        for c in range(width):
            push_if_border_free(0, c)
            push_if_border_free(height - 1, c)

        for r in range(height):
            push_if_border_free(r, 0)
            push_if_border_free(r, width - 1)

        # BFS (breath first algorithm)
        while q:
            r, c = q.popleft()
            for dr, dc in ((1, 0), (-1, 0), (0,1), (0, -1)):
                rr, cc = r + dr, c + dc
                if 0 <= rr < height and 0 <= cc < width:
                    i = idx(rr, cc)
                    if free[i] and not outside[i]:
                        outside[i] = True
                        q.append((rr, cc))


        # Generate waypoints from interior (free, spaces inside the walls)
        waypoints = []
        for r in range(0, height, step):
            c_range = range(0, width, step) if (r // step) % 2 == 0 else range(width - 1, -1, -step)
            for c in c_range:
                i = idx(r, c)

                if i >= len(data):
                    continue

                if free[i] and not outside[i]:
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
