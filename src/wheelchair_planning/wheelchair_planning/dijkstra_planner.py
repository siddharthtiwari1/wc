#!/usr/bin/env python3

"""
Robust Dijkstra Path Planner for Wheelchair Navigation
=======================================================
- Uses /map topic (from map_server via localization)
- Tries multiple robot frame names (base_link, base_footprint)
- Handles TF lookup with timeout and retries
- Diagonal movement support for smoother paths
- Proper error handling throughout
"""

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from nav_msgs.msg import OccupancyGrid, Path
from geometry_msgs.msg import PoseStamped, Pose
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from tf2_ros import Buffer, TransformListener, LookupException, ConnectivityException, ExtrapolationException
from queue import PriorityQueue
import math


class GraphNode:
    def __init__(self, x, y, cost=0, prev=None):
        self.x = x
        self.y = y
        self.cost = cost
        self.prev = prev

    def __lt__(self, other):
        return self.cost < other.cost

    def __eq__(self, other):
        return self.x == other.x and self.y == other.y

    def __hash__(self):
        return hash((self.x, self.y))

    def __add__(self, other):
        return GraphNode(self.x + other[0], self.y + other[1])


class DijkstraPlanner(Node):
    def __init__(self):
        super().__init__("dijkstra_planner")

        # Declare parameters
        self.declare_parameter('robot_frame', 'base_link')
        self.declare_parameter('map_topic', '/map')
        self.declare_parameter('goal_topic', '/goal_pose')
        self.declare_parameter('tf_timeout', 1.0)
        self.declare_parameter('occupied_threshold', 65)  # Cells with value >= this are obstacles
        self.declare_parameter('allow_diagonal', True)

        self.robot_frame = self.get_parameter('robot_frame').value
        self.map_topic = self.get_parameter('map_topic').value
        self.goal_topic = self.get_parameter('goal_topic').value
        self.tf_timeout = self.get_parameter('tf_timeout').value
        self.occupied_threshold = self.get_parameter('occupied_threshold').value
        self.allow_diagonal = self.get_parameter('allow_diagonal').value

        # TF2 buffer and listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # QoS for map - transient local to get latched map
        map_qos = QoSProfile(depth=1)
        map_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        map_qos.reliability = ReliabilityPolicy.RELIABLE

        # Subscribers
        self.map_sub = self.create_subscription(
            OccupancyGrid, self.map_topic, self.map_callback, map_qos
        )
        self.pose_sub = self.create_subscription(
            PoseStamped, self.goal_topic, self.goal_callback, 10
        )

        # Publishers
        self.path_pub = self.create_publisher(Path, "/dijkstra/path", 10)
        self.map_pub = self.create_publisher(OccupancyGrid, "/dijkstra/visited_map", 10)

        # State
        self.map_ = None
        self.visited_map_ = OccupancyGrid()
        self.map_frame = "map"

        # Robot frame candidates to try
        self.robot_frame_candidates = ['base_link', 'base_footprint', 'base']

        self.get_logger().info(f"Dijkstra Planner initialized")
        self.get_logger().info(f"  Map topic: {self.map_topic}")
        self.get_logger().info(f"  Goal topic: {self.goal_topic}")
        self.get_logger().info(f"  Robot frame: {self.robot_frame} (will try alternatives if needed)")
        self.get_logger().info("Waiting for map...")

    def map_callback(self, map_msg: OccupancyGrid):
        self.map_ = map_msg
        self.map_frame = map_msg.header.frame_id
        self.visited_map_.header.frame_id = map_msg.header.frame_id
        self.visited_map_.info = map_msg.info
        self.visited_map_.data = [-1] * (map_msg.info.height * map_msg.info.width)
        self.get_logger().info(
            f"Map received: {map_msg.info.width}x{map_msg.info.height} @ {map_msg.info.resolution}m/cell, "
            f"frame: {self.map_frame}"
        )

    def get_robot_pose(self):
        """Try to get robot pose using multiple frame candidates."""
        if self.map_ is None:
            return None

        # Try the configured frame first, then alternatives
        frames_to_try = [self.robot_frame] + [f for f in self.robot_frame_candidates if f != self.robot_frame]

        for frame in frames_to_try:
            try:
                transform = self.tf_buffer.lookup_transform(
                    self.map_frame,
                    frame,
                    rclpy.time.Time(),
                    timeout=Duration(seconds=self.tf_timeout)
                )

                pose = Pose()
                pose.position.x = transform.transform.translation.x
                pose.position.y = transform.transform.translation.y
                pose.position.z = transform.transform.translation.z
                pose.orientation = transform.transform.rotation

                if frame != self.robot_frame:
                    self.get_logger().info(f"Using robot frame: {frame} (instead of {self.robot_frame})")
                    self.robot_frame = frame

                return pose

            except (LookupException, ConnectivityException, ExtrapolationException) as e:
                self.get_logger().debug(f"TF lookup failed for {self.map_frame} -> {frame}: {e}")
                continue

        return None

    def goal_callback(self, pose: PoseStamped):
        if self.map_ is None:
            self.get_logger().error("No map received! Waiting for map on topic: " + self.map_topic)
            return

        # Reset visited map
        self.visited_map_.data = [-1] * (self.visited_map_.info.height * self.visited_map_.info.width)

        # Get robot pose
        robot_pose = self.get_robot_pose()
        if robot_pose is None:
            self.get_logger().error(
                f"Could not get robot pose! Tried frames: {self.robot_frame_candidates}. "
                f"Make sure AMCL is publishing map->odom transform."
            )
            return

        self.get_logger().info(
            f"Planning from ({robot_pose.position.x:.2f}, {robot_pose.position.y:.2f}) "
            f"to ({pose.pose.position.x:.2f}, {pose.pose.position.y:.2f})"
        )

        # Plan path
        path = self.plan(robot_pose, pose.pose)

        if path.poses:
            self.get_logger().info(f"Path found with {len(path.poses)} waypoints!")
            self.path_pub.publish(path)
        else:
            self.get_logger().warn("No path found to the goal.")

    def plan(self, start: Pose, goal: Pose):
        """Dijkstra's algorithm for path planning."""

        # Movement directions: 4-connected or 8-connected
        if self.allow_diagonal:
            # 8-connected with diagonal costs
            explore_directions = [
                (-1, 0, 1.0), (1, 0, 1.0), (0, -1, 1.0), (0, 1, 1.0),  # Cardinal
                (-1, -1, 1.414), (-1, 1, 1.414), (1, -1, 1.414), (1, 1, 1.414)  # Diagonal
            ]
        else:
            explore_directions = [(-1, 0, 1.0), (1, 0, 1.0), (0, -1, 1.0), (0, 1, 1.0)]

        pending_nodes = PriorityQueue()
        visited_nodes = set()

        start_node = self.world_to_grid(start)
        goal_node = self.world_to_grid(goal)

        # Validate start and goal
        if not self.pose_on_map(start_node):
            self.get_logger().error(f"Start position ({start_node.x}, {start_node.y}) is outside map!")
            return Path()

        if not self.pose_on_map(goal_node):
            self.get_logger().error(f"Goal position ({goal_node.x}, {goal_node.y}) is outside map!")
            return Path()

        if self.is_occupied(start_node):
            self.get_logger().warn("Start position is in an occupied cell!")

        if self.is_occupied(goal_node):
            self.get_logger().error("Goal position is in an occupied cell!")
            return Path()

        pending_nodes.put(start_node)
        goal_reached = False
        final_node = None

        iterations = 0
        max_iterations = self.map_.info.width * self.map_.info.height

        while not pending_nodes.empty() and rclpy.ok() and iterations < max_iterations:
            iterations += 1
            active_node = pending_nodes.get()

            # Skip if already visited
            if active_node in visited_nodes:
                continue

            visited_nodes.add(active_node)

            # Goal check
            if active_node == goal_node:
                goal_reached = True
                final_node = active_node
                break

            # Explore neighbors
            for dir_x, dir_y, move_cost in explore_directions:
                new_node = GraphNode(active_node.x + dir_x, active_node.y + dir_y)

                if new_node in visited_nodes:
                    continue

                if not self.pose_on_map(new_node):
                    continue

                if self.is_occupied(new_node):
                    continue

                # Calculate cost (base movement cost + any cell cost)
                cell_value = self.map_.data[self.pose_to_cell(new_node)]
                cell_cost = max(0, cell_value) if cell_value >= 0 else 0
                new_node.cost = active_node.cost + move_cost + cell_cost * 0.1
                new_node.prev = active_node

                pending_nodes.put(new_node)

            # Update visualization (throttled)
            if iterations % 100 == 0:
                self.visited_map_.data[self.pose_to_cell(active_node)] = 50
                self.map_pub.publish(self.visited_map_)

        # Mark final visited state
        for node in visited_nodes:
            if self.pose_on_map(node):
                self.visited_map_.data[self.pose_to_cell(node)] = 50
        self.map_pub.publish(self.visited_map_)

        # Build path
        path = Path()
        path.header.frame_id = self.map_frame
        path.header.stamp = self.get_clock().now().to_msg()

        if goal_reached and final_node:
            # Backtrack from goal to start
            current = final_node
            while current is not None:
                pose_stamped = PoseStamped()
                pose_stamped.header.frame_id = self.map_frame
                pose_stamped.pose = self.grid_to_world(current)
                path.poses.append(pose_stamped)
                current = current.prev

            path.poses.reverse()
            self.get_logger().info(f"Dijkstra explored {iterations} cells")

        return path

    def pose_on_map(self, node: GraphNode) -> bool:
        return 0 <= node.x < self.map_.info.width and 0 <= node.y < self.map_.info.height

    def is_occupied(self, node: GraphNode) -> bool:
        """Check if a cell is occupied or unknown."""
        cell_value = self.map_.data[self.pose_to_cell(node)]
        # -1 = unknown, 0 = free, 100 = occupied
        return cell_value < 0 or cell_value >= self.occupied_threshold

    def world_to_grid(self, pose: Pose) -> GraphNode:
        grid_x = int((pose.position.x - self.map_.info.origin.position.x) / self.map_.info.resolution)
        grid_y = int((pose.position.y - self.map_.info.origin.position.y) / self.map_.info.resolution)
        return GraphNode(grid_x, grid_y)

    def grid_to_world(self, node: GraphNode) -> Pose:
        pose = Pose()
        pose.position.x = node.x * self.map_.info.resolution + self.map_.info.origin.position.x + self.map_.info.resolution / 2
        pose.position.y = node.y * self.map_.info.resolution + self.map_.info.origin.position.y + self.map_.info.resolution / 2
        pose.position.z = 0.0
        pose.orientation.w = 1.0
        return pose

    def pose_to_cell(self, node: GraphNode) -> int:
        return node.y * self.map_.info.width + node.x


def main(args=None):
    rclpy.init(args=args)
    node = DijkstraPlanner()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
