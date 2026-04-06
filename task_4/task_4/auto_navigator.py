#!/usr/bin/env python3
import os
import math
import heapq
import yaml
from collections import deque
import numpy as np
from PIL import Image
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Twist
from std_msgs.msg import Float32
from tf_transformations import euler_from_quaternion

def get_yaw_from_quaternion(q): # q=(x,y,z) The goal is to get the yaw angle (heading) from the quaternion

    """
    q: geometry_msgs.msg.Quaternion
    Returns yaw (rotation around Z) in radians
    """
    # Convert quaternion to tuple
    quat = [q.x, q.y, q.z, q.w]
    roll, pitch, yaw = euler_from_quaternion(quat)
    return yaw # we only require yaw since there is no pitch roll involved for ttbot

def normalize_angle(angle):
    # To maintain the angle in the specified range
    while angle > math.pi:
        angle -= 2.0 * math.pi
    # If the angle is larger than π (180°), we subtract 2π until it comes back into range
    while angle < -math.pi:
        angle += 2.0 * math.pi
    # If the angle is less than -π (-180°), we add 2π until it’s inside the range
    return angle

class GraphNode:
    # represents a single node (vertex) in a graph   
    def __init__(self, name):
        self.name = name  # unique identifier
        self.neighbors = [] # A list of neighbour nodes the primary node connects to
        self.costs = [] # A list of weights

    def add_edges(self, nums, ws): # nums is a list of nodes & ws is their weights
        self.neighbors.extend(nums) # extends nums to neighbors list
        self.costs.extend(ws) # extends weights to ws list

class GridGraph: # holds all the nodes
    def __init__(self):
        self.nodes = {}  # The graph is represented as a dictionary (self.nodes)


class AStar:  # plan a path from a start node to a goal node using the A* algorithm
    def __init__(self, graph: GridGraph):  # uses GrdiGraph from previous code
        self.graph = graph # stores the graph
        self.g = {} # A dictionary of nodes and their assosiated costs
        self.h = {} # A dictionary of heurestic function of the nodes
        self.prev = {} # stores the previous node along the best path

    def _reset(self): # prepares for a brand new search
        self.g = {n: float('inf') for n in self.graph.nodes} # sets distance from start to every node to infinity initially
        self.h = {n: 0.0 for n in self.graph.nodes} # sets heurestic estimate to 0
        self.prev = {n: None for n in self.graph.nodes} # sets previous node to None

    def _calc_heuristic(self, goal): # computes heurestic distance of each node to the goal
        goal_row, goal_column= map(int, goal.split(',')) # Converts goal string to integers
        for name in self.graph.nodes: # Converts each node name "row,column" into integers r (row), c (col)
            row, column = map(int, name.split(','))
            self.h[name] = math.hypot(goal_row - row, goal_column - column) # distance between current node and goal defined by sqrt(dx^2 + dy^2)

    def plan(self, start_name, goal): # Finds shortest distance between start and goal
        if start_name not in self.graph.nodes or goal not in self.graph.nodes: # if either the start or goal node doesn’t exist in the graph, exit
            return
        self._reset() 
        self._calc_heuristic(goal)
        self.g[start_name] = 0.0 # The cost from start to itself is 0
        open_heap = [(self.h[start_name], start_name)] # priority queue of nodes to explore, sorted by f(n) = g + h
        closed = set() # set of already visited nodes

        while open_heap: 
            _, cur = heapq.heappop(open_heap) # Pop node with lowest f-score from the heap
            if cur in closed:
                continue # Skip nodes we’ve already fully processed
            closed.add(cur) # Add current node to closed
            if cur == goal:
                break # If we reached the goal, stop searching
            node = self.graph.nodes[cur]
            for neighbor, w in zip(node.neighbors, node.costs): 
                neighbor_name = neighbor.name
                newg = self.g[cur] + float(w) # newg = cost to reach neighbor through current node.
                if newg < self.g[neighbor_name]:  
                # if the new neighbor is cheaper update its g and its previous node
                    self.g[neighbor_name] = newg
                    self.prev[neighbor_name] = cur
                    heapq.heappush(open_heap, (newg + self.h[neighbor_name], neighbor_name)) # Push neighbor onto the heap with priority

    def reconstruct(self, start_name, goal): # traces the shortest path from goal back to start
        if self.g.get(goal, float('inf')) == float('inf'):
            return [] # if the goal’s g value is still inf, there’s no path → return empty list.
        path = []
        cur = goal
        while cur is not None:
            path.append(cur) # adding nodes to path
            cur = self.prev[cur]
        path.reverse() # Reverse the list so it goes from start → goal.
        return path


class Navigation(Node): 
    def __init__(self, node_name='Navigation'):
        super().__init__(node_name)
        self.get_logger().info("[INIT] Navigation started")

        # Path planning / robot state variables
        self.path = Path()
        self.goal_pose = PoseStamped()
        self.ttbot_pose = PoseStamped()
        self.start_time = 0.0
        
        # Subscribers
        self.create_subscription(PoseStamped, '/move_base_simple/goal', self.__goal_pose_cbk, 10)
        self.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', self.__ttbot_pose_cbk, 10)
        
        # Publishers
        self.path_pub = self.create_publisher(Path, 'global_plan', 10)
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.calc_time_pub = self.create_publisher(Float32, 'astar_time', 10)
        self.rate = self.create_rate(10)

        # Map settings & robot footprint
        self.map_yaml = '/home/ashish/sim_ws/src/task_4/maps/sync_classroom_map.yaml'
        self.flip_y = True
        self.treat_unknown_as_occupied = True
        self.robot_radius = 0.19
        self.extra_inflation = 0.09

        # Planner / follower parameters
        self.lookahead = 0.30
        self.wp_dist = 0.12
        self.goal_tol = 0.12
        self.max_lin = 0.07
        self.max_ang = 0.6
        self.rotate_threshold = 0.6

        # Map and graph state
        self.map_loaded = False
        self.map_occ = None
        self.inflated = None
        self.res = 0.05
        self.ox = 0.0
        self.oy = 0.0
        self.W = 0
        self.H = 0
        self.graph = None
        self.astar = None
        self.arrived = False

        # Velocity smoothing
        self.v_last = 0.0
        self.w_last = 0.0
        self.smoothing_alpha = 0.45

        # Load map & build graph
        self._load_map_from_yaml(self.map_yaml)
        if self.map_loaded:
            self.get_logger().info("[READY] Map & graph ready")
        else:
            self.get_logger().error("[READY] Map not loaded")

    def __goal_pose_cbk(self, data: PoseStamped): # Updates the target goal for the robot whenever a new goal is sent from RViz
        self.goal_pose = data
        self.arrived = False
        self.get_logger().info(f"New goal: ({data.pose.position.x:.3f}, {data.pose.position.y:.3f})")

    def __ttbot_pose_cbk(self, data: PoseWithCovarianceStamped): # Updates the current position of the robot according to AMCL (Adaptive Monte Carlo Localization)
        ps = PoseStamped()
        ps.header = data.header
        ps.pose = data.pose.pose
        self.ttbot_pose = ps
        self.get_logger().debug(f"AMCL update: ({ps.pose.position.x:.3f}, {ps.pose.position.y:.3f})")

    def _load_map_from_yaml(self, yaml_path: str): # Load a map and prepare it for path planning
        if not yaml_path or not os.path.isfile(yaml_path):
            self.get_logger().error(f"Map YAML not found: {yaml_path}")
            return

        with open(yaml_path, 'r') as f:
            y = yaml.safe_load(f)

        image_path = y.get('image')
        if not image_path:
            self.get_logger().error("YAML missing 'image' field")
            return
        if not os.path.isabs(image_path):
            image_path = os.path.join(os.path.dirname(yaml_path), image_path)

        self.res = float(y.get('resolution', 0.05))
        origin = y.get('origin', [0.0, 0.0, 0.0])
        self.ox, self.oy = float(origin[0]), float(origin[1])

        negate = int(y.get('negate', 0))
        occ_th_yaml = float(y.get('occupied_thresh', 0.65))
        free_th_yaml = float(y.get('free_thresh', 0.25))
        mode = str(y.get('mode', 'trinary'))

        img = Image.open(image_path).convert('L')
        W, H = img.size
        self.W, self.H = W, H
        pix = np.array(img, dtype=np.float32) / 255.0  # 0..1

        if negate == 1:
            pix = 1.0 - pix

        if self.flip_y:
            pix = np.flipud(pix)

        if mode == 'trinary':
            occ = np.full((H, W), -1, dtype=np.int16)
            occ[pix >= free_th_yaml] = 0
            occ[pix <= occ_th_yaml] = 100
        else:
            occ = np.round(100.0 * (1.0 - pix)).astype(np.int16)

        if self.treat_unknown_as_occupied:
            occ[occ < 0] = 100

        free_pre = int(np.count_nonzero(occ == 0))
        occ_pre = int(np.count_nonzero(occ == 100))
        self.get_logger().info(f"[MAP] Loading: {yaml_path}")
        self.get_logger().info(f"[MAP] Size={W}x{H}, res={self.res:.3f}, origin=({self.ox:.2f},{self.oy:.2f})")
        self.get_logger().info(f"[MAP] Free cells: {free_pre}, Occupied: {occ_pre}")

        self.map_occ = occ
        self.inflated = self._inflate(occ)
        free_after = int(np.count_nonzero(self.inflated == 0))
        occ_after = int(np.count_nonzero(self.inflated == 100))
        self.get_logger().info(f"[MAP] After inflation free={free_after}, occ={occ_after}")

        self.graph = self._build_graph_from_inflated(self.inflated)
        self.astar = AStar(self.graph)
        self.map_loaded = True
        self.get_logger().info(f"[GRAPH] nodes={len(self.graph.nodes)}")

    def _inflate(self, occ_grid: np.ndarray): # Inflate obstacles so the robot doesn't collide
        H, W = occ_grid.shape
        base = np.where(occ_grid >= 50, 100, 0).astype(np.uint8)
        robot_r = float(self.robot_radius)
        pad = float(self.extra_inflation)
        r_cells = int(math.ceil((robot_r + pad) / self.res))
        if r_cells <= 0:
            return base
        yy, xx = np.ogrid[-r_cells:r_cells+1, -r_cells:r_cells+1]
        disk = (xx*xx + yy*yy) <= (r_cells*r_cells)
        dil = base.copy()
        # slice-based expansion (no wrapping)
        for dy in range(-r_cells, r_cells+1):
            for dx in range(-r_cells, r_cells+1):
                if not disk[dy + r_cells, dx + r_cells]:
                    continue
                src_y0 = max(0, -dy)
                src_y1 = H - max(0, dy)
                src_x0 = max(0, -dx)
                src_x1 = W - max(0, dx)
                dst_y0 = max(0, dy)
                dst_y1 = H - max(0, -dy)
                dst_x0 = max(0, dx)
                dst_x1 = W - max(0, -dx)
                # careful views (no wrap)
                dst = dil[dst_y0:dst_y1, dst_x0:dst_x1]
                src = base[src_y0:src_y1, src_x0:src_x1]
                np.maximum(dst, src, out=dst)
        return dil

    def _build_graph_from_inflated(self, inflated): # Convert free cells into a graph for A*
        H, W = inflated.shape
        g = GridGraph()
        for r in range(H):
            for c in range(W):
                if inflated[r, c] == 0:
                    g.nodes[f"{r},{c}"] = GraphNode(f"{r},{c}")
        nbrs = [
            (-1,  0, 1.0), ( 1,  0, 1.0), ( 0, -1, 1.0), ( 0,  1, 1.0),
            (-1, -1, math.sqrt(2.0)), (-1,  1, math.sqrt(2.0)),
            ( 1, -1, math.sqrt(2.0)), ( 1,  1, math.sqrt(2.0))
        ]
        for r in range(H):
            for c in range(W):
                if inflated[r, c] != 0:
                    continue
                parent = g.nodes.get(f"{r},{c}")
                if parent is None:
                    continue
                neighbors, weights = [], []
                for dr, dc, w in nbrs:
                    nr, nc = r + dr, c + dc
                    if 0 <= nr < H and 0 <= nc < W and inflated[nr, nc] == 0:
                        neighbors.append(g.nodes[f"{nr},{nc}"])
                        weights.append(w)
                if neighbors:
                    parent.add_edges(neighbors, weights)
        return g

    def _in_map_world(self, x, y):
        return (self.ox <= x < self.ox + self.W * self.res) and (self.oy <= y < self.oy + self.H * self.res)

    def _nearest_free(self, col, row): # If a cell is occupied, find the nearest free cell
        H, W = self.H, self.W
        if 0 <= row < H and 0 <= col < W and self.inflated[row, col] == 0:
            return col, row
        visited = np.zeros((H, W), dtype=np.uint8)
        q = deque()
        c0 = max(0, min(W-1, col))
        r0 = max(0, min(H-1, row))
        q.append((c0, r0))
        visited[r0, c0] = 1
        while q:
            c, r = q.popleft()
            if self.inflated[r, c] == 0:
                return c, r
            for dc, dr in [(-1,0),(1,0),(0,-1),(0,1),(-1,-1),(-1,1),(1,-1),(1,1)]:
                nc, nr = c+dc, r+dr
                if 0 <= nc < W and 0 <= nr < H and not visited[nr, nc]:
                    visited[nr, nc] = 1
                    q.append((nc, nr))
        # fallback clamp
        nc = max(0, min(W-1, col))
        nr = max(0, min(H-1, row))
        return nc, nr

    def world_to_grid(self, x, y):
        col = int((x - self.ox) / self.res)
        row = int((y - self.oy) / self.res)
        col = max(0, min(self.W - 1, col))
        row = max(0, min(self.H - 1, row))
        return col, row

    def grid_to_world(self, col, row):
        x = self.ox + (col + 0.5) * self.res
        y = self.oy + (row + 0.5) * self.res
        return x, y

    def a_star_path_planner(self, start_pose, end_pose): # Compute a global path from the robot’s current position to a goal using A*
        path = Path()
        path.header.frame_id = end_pose.header.frame_id or 'map'
        self.get_logger().info('A* planner called.')
        self.start_time = self.get_clock().now().nanoseconds * 1e-9  # required
        sx_w, sy_w = start_pose.pose.position.x, start_pose.pose.position.y
        gx_w, gy_w = end_pose.pose.position.x, end_pose.pose.position.y
        # DEBUG: print map & node info immediately
        self.get_logger().info(f"[DEBUG] map_loaded={self.map_loaded}, graph_nodes={len(self.graph.nodes) if self.graph else 0}")
        self.get_logger().info(f"[DEBUG] start_world=({sx_w:.3f},{sy_w:.3f}), goal_world=({gx_w:.3f},{gy_w:.3f})")

        if not self.map_loaded or self.graph is None:
            self.get_logger().error("[PLAN] Map/graph not ready")
            # always publish astar_time for grader
            tmsg = Float32()
            tmsg.data = float(self.get_clock().now().nanoseconds * 1e-9 - self.start_time)
            self.calc_time_pub.publish(tmsg)
            return path

        if not self._in_map_world(gx_w, gy_w): # Check if a world coordinate is inside map boundaries.
            self.get_logger().error(f"[PLAN] Goal outside map: ({gx_w:.3f},{gy_w:.3f})")
            tmsg = Float32()
            tmsg.data = float(self.get_clock().now().nanoseconds * 1e-9 - self.start_time)
            self.calc_time_pub.publish(tmsg)
            return path

        sc, sr = self.world_to_grid(sx_w, sy_w)
        goal_column, goal_row= self.world_to_grid(gx_w, gy_w)

        # snap to nearest free
        sc, sr = self._nearest_free(sc, sr)
        goal_column, goal_row= self._nearest_free(goal_column, goal_row)
        s_name = f"{sr},{sc}"
        g_name = f"{goal_row},{goal_column}"

        self.get_logger().info(f"[DEBUG] start grid (col,row)=({sc},{sr}), goal grid (col,row)=({goal_column},{goal_row})")
        self.get_logger().info(f"[DEBUG] s_name={s_name} in graph? {s_name in self.graph.nodes}")
        self.get_logger().info(f"[DEBUG] g_name={g_name} in graph? {g_name in self.graph.nodes}")

        if s_name not in self.graph.nodes or g_name not in self.graph.nodes:
            self.get_logger().error("[PLAN] start or goal not in graph after snapping")
            tmsg = Float32()
            tmsg.data = float(self.get_clock().now().nanoseconds * 1e-9 - self.start_time)
            self.calc_time_pub.publish(tmsg)
            return path

        # run A*
        self.astar.plan(s_name, g_name)
        names = self.astar.reconstruct(s_name, g_name)
        if not names:
            self.get_logger().warn("[PLAN] A* returned empty path")
            tmsg = Float32()
            tmsg.data = float(self.get_clock().now().nanoseconds * 1e-9 - self.start_time)
            self.calc_time_pub.publish(tmsg)
            return path

        for nm in names:
            r, c = map(int, nm.split(','))
            wx, wy = self.grid_to_world(c, r)
            ps = PoseStamped()
            ps.header.frame_id = path.header.frame_id
            ps.pose.position.x = float(wx)
            ps.pose.position.y = float(wy)
            ps.pose.orientation.w = 1.0
            path.poses.append(ps)

        # publish astar time (required)
        tmsg = Float32()
        tmsg.data = float(self.get_clock().now().nanoseconds * 1e-9 - self.start_time)
        self.calc_time_pub.publish(tmsg)

        self.path_pub.publish(path)
        self.get_logger().info(f"[PLAN] Published path with {len(path.poses)} poses (t={tmsg.data:.6f}s)")
        return path

    def get_path_idx(self, path, vehicle_pose): # Find the next waypoint along the path for the robot to follow
        if not path.poses:
            return 0
        x = vehicle_pose.pose.position.x
        y = vehicle_pose.pose.position.y
        for i, ps in enumerate(path.poses):
            dx = ps.pose.position.x - x
            dy = ps.pose.position.y - y
            if math.hypot(dx, dy) >= self.lookahead:
                return i
        return len(path.poses) - 1

    def path_follower(self, vehicle_pose, current_goal_pose): # Compute linear (v_cmd) and angular (w_cmd) velocity commands to move toward a waypoint
        robot_x = vehicle_pose.pose.position.x
        robot_y = vehicle_pose.pose.position.y
        gx = current_goal_pose.pose.position.x
        gy = current_goal_pose.pose.position.y
        ryaw = get_yaw_from_quaternion(vehicle_pose.pose.orientation)
        dx = gx - robot_x
        dy = gy - robot_y
        dist = math.hypot(dx, dy)
        tgt_yaw = math.atan2(dy, dx)
        yaw_err = normalize_angle(tgt_yaw - ryaw)

        if abs(yaw_err) > self.rotate_threshold:
            v_cmd = 0.0
            w_cmd = max(-self.max_ang, min(self.max_ang, 1.5 * yaw_err))
            self.v_last = (1.0 - self.smoothing_alpha) * self.v_last + self.smoothing_alpha * v_cmd
            self.w_last = (1.0 - self.smoothing_alpha) * self.w_last + self.smoothing_alpha * w_cmd
            return self.v_last, self.w_last

        v_cmd = self.max_lin * max(0.0, math.cos(yaw_err))
        w_cmd = max(-self.max_ang, min(self.max_ang, 1.2 * yaw_err))

        if dist < self.goal_tol:
            v_cmd = 0.0
            w_cmd = 0.0

        self.v_last = (1.0 - self.smoothing_alpha) * self.v_last + self.smoothing_alpha * v_cmd
        self.w_last = (1.0 - self.smoothing_alpha) * self.w_last + self.smoothing_alpha * w_cmd

        return self.v_last, self.w_last

    def move_ttbot(self, speed, heading):
        cmd = Twist()
        cmd.linear.x = float(speed)
        cmd.angular.z = float(heading)
        self.cmd_vel_pub.publish(cmd)

    def run(self): # Main control loop that runs continuously
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)

            # need both poses to be valid
            if self.goal_pose.header.stamp.sec == 0 or self.ttbot_pose.header.stamp.sec == 0:
                continue

            robot_x = self.ttbot_pose.pose.position.x
            robot_y = self.ttbot_pose.pose.position.y
            gx = self.goal_pose.pose.position.x
            gy = self.goal_pose.pose.position.y
            if math.hypot(gx - robot_x, gy - robot_y) < self.goal_tol:
                if not self.arrived:
                    self.arrived = True
                    self.move_ttbot(0.0, 0.0)
                    self.get_logger().info("[RUN] Goal reached. Stopping.")
                continue

            path = self.a_star_path_planner(self.ttbot_pose, self.goal_pose)
            if not path.poses:
                self.move_ttbot(0.0, 0.0)
                continue

            idx = self.get_path_idx(path, self.ttbot_pose)
            current_goal = path.poses[idx]
            vel, Heading = self.path_follower(self.ttbot_pose, current_goal)
            self.move_ttbot(vel, Heading)

def main(args=None):
    rclpy.init(args=args)
    nav = Navigation(node_name='Navigation')
    try:
        nav.run()
    except KeyboardInterrupt:
        pass
    finally:
        try:
            nav.destroy_node()
        finally:
            if rclpy.ok():
                rclpy.shutdown()


if __name__ == "__main__":
    main()
