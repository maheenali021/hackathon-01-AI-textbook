---
sidebar_position: 3
title: "Chapter 3: Navigation2 Path Planning and Execution"
---

# Navigation2 Path Planning and Execution

## Introduction to Navigation2

Navigation2 is the official navigation framework for ROS 2, providing path planning, obstacle avoidance, and navigation execution capabilities for mobile robots. It builds upon the lessons learned from ROS 1's navigation stack with improved architecture and performance.

## Navigation2 Architecture

### Core Components

Navigation2 follows a modular architecture with several key components:

1. **Lifecycle Manager**: Manages the lifecycle of navigation components
2. **Local Planner**: Handles local path following and obstacle avoidance
3. **Global Planner**: Computes global paths from start to goal
4. **Controller**: Executes trajectory following
5. **Recovery**: Handles navigation recovery behaviors

### Parameter Configuration

Navigation2 uses YAML configuration files to define behavior:

```yaml
# bt_navigator_params.yaml
bt_navigator:
  ros__parameters:
    global_frame: map
    robot_base_frame: base_link
    odom_topic: /odom
    bt_loop_duration: 10
    default_server_timeout: 20
    enable_groot_monitoring: True
    groot_zmq_publisher_port: 1666
    groot_zmq_server_port: 1667
    # Specify the path where the BT XML files are located
    plugin_lib_names:
    - nav2_compute_path_to_pose_action_bt_node
    - nav2_follow_path_action_bt_node
    - nav2_back_up_action_bt_node
    - nav2_spin_action_bt_node
    - nav2_wait_action_bt_node
    - nav2_clear_costmap_service_bt_node
    - nav2_is_stuck_condition_bt_node
    - nav2_goal_reached_condition_bt_node
    - nav2_goal_updated_condition_bt_node
    - nav2_initial_pose_received_condition_bt_node
    - nav2_reinitialize_global_localization_service_bt_node
    - nav2_rate_controller_bt_node
    - nav2_distance_controller_bt_node
    - nav2_speed_controller_bt_node
    - nav2_truncate_path_action_bt_node
    - nav2_goal_updater_node_bt_node
    - nav2_recovery_node_bt_node
    - nav2_pipeline_sequence_bt_node
    - nav2_round_robin_node_bt_node
    - nav2_transform_available_condition_bt_node
    - nav2_time_expired_condition_bt_node
    - nav2_path_expiring_timer_condition
    - nav2_distance_traveled_condition_bt_node
    - nav2_single_trigger_bt_node
    - nav2_is_battery_low_condition_bt_node
    - nav2_navigate_to_pose_action_bt_node
    - nav2_remove_passed_goals_action_bt_node
    - nav2_planner_selector_bt_node
    - nav2_controller_selector_bt_node
    - nav2_goal_checker_selector_bt_node
```

## Global Path Planning

### NavFn Planner

The NavFn planner computes global paths using the navigation function approach:

```python
import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid, Path
from geometry_msgs.msg import PoseStamped, Point
from nav2_msgs.srv import GetCostmap
import numpy as np

class GlobalPathPlanner(Node):
    def __init__(self):
        super().__init__('global_path_planner')

        # Subscribers
        self.map_sub = self.create_subscription(
            OccupancyGrid, 'map', self.map_callback, 10)

        # Publishers
        self.path_pub = self.create_publisher(Path, 'global_plan', 10)

        # Service client for costmap
        self.costmap_client = self.create_client(GetCostmap, 'costmap/get_costmap')

        # Internal state
        self.map_data = None
        self.map_resolution = 0.05  # meters per pixel
        self.origin = [0, 0, 0]  # x, y, theta

        # Timer for path planning
        self.path_timer = self.create_timer(0.5, self.plan_path_if_needed)

    def map_callback(self, msg):
        """Process incoming map data"""
        self.map_data = msg
        self.map_resolution = msg.info.resolution
        self.origin = [
            msg.info.origin.position.x,
            msg.info.origin.position.y,
            msg.info.origin.orientation.z  # Simplified
        ]

    def plan_path_if_needed(self):
        """Plan path if needed"""
        # This would normally be triggered by a navigation goal
        pass

    def compute_path(self, start, goal):
        """Compute path using Dijkstra's algorithm (NavFn approach)"""
        if not self.map_data:
            return None

        # Convert world coordinates to map indices
        start_idx = self.world_to_map(start)
        goal_idx = self.world_to_map(goal)

        if not start_idx or not goal_idx:
            return None

        # Create costmap from occupancy grid
        costmap = self.create_costmap_from_occupancy(self.map_data)

        # Run path planning algorithm (simplified Dijkstra's)
        path_indices = self.dijkstra_pathfinding(costmap, start_idx, goal_idx)

        if path_indices:
            # Convert path back to world coordinates
            path = self.indices_to_path(path_indices)
            return path

        return None

    def world_to_map(self, point):
        """Convert world coordinates to map indices"""
        if not self.map_data:
            return None

        x = int((point.x - self.map_data.info.origin.position.x) / self.map_data.info.resolution)
        y = int((point.y - self.map_data.info.origin.position.y) / self.map_data.info.resolution)

        # Check bounds
        if x < 0 or x >= self.map_data.info.width or y < 0 or y >= self.map_data.info.height:
            return None

        return (y, x)  # Note: row, col (y, x)

    def map_to_world(self, row, col):
        """Convert map indices to world coordinates"""
        x = col * self.map_data.info.resolution + self.map_data.info.origin.position.x
        y = row * self.map_data.info.resolution + self.map_data.info.origin.position.y
        return (x, y)

    def create_costmap_from_occupancy(self, occupancy_grid):
        """Create costmap from occupancy grid"""
        width = occupancy_grid.info.width
        height = occupancy_grid.info.height
        data = occupancy_grid.data

        # Reshape the 1D array to 2D
        costmap = np.array(data).reshape(height, width)

        # Convert occupancy values to costs
        # Free space: 0, Occupied: 100, Unknown: -1
        # In costmap: higher values mean higher cost
        costmap_processed = np.copy(costmap)
        costmap_processed[costmap_processed == -1] = 50  # Unknown space has medium cost
        costmap_processed[costmap_processed == 0] = 0    # Free space has no cost
        costmap_processed[costmap_processed > 0] = 100   # Occupied space has high cost

        return costmap_processed

    def dijkstra_pathfinding(self, costmap, start_idx, goal_idx):
        """Simplified Dijkstra's algorithm for pathfinding"""
        import heapq

        height, width = costmap.shape
        start_row, start_col = start_idx
        goal_row, goal_col = goal_idx

        # Directions: up, down, left, right, and diagonals
        directions = [(-1,-1), (-1,0), (-1,1), (0,-1), (0,1), (1,-1), (1,0), (1,1)]
        direction_costs = [1.414, 1.0, 1.414, 1.0, 1.0, 1.414, 1.0, 1.414]

        # Priority queue: (cost, row, col)
        pq = [(0, start_row, start_col)]
        visited = set()
        predecessors = {}

        while pq:
            cost, row, col = heapq.heappop(pq)

            if (row, col) in visited:
                continue

            visited.add((row, col))

            if (row, col) == (goal_row, goal_col):
                # Found the goal, reconstruct path
                return self.reconstruct_path(predecessors, start_idx, goal_idx)

            # Explore neighbors
            for i, (dr, dc) in enumerate(directions):
                new_row, new_col = row + dr, col + dc

                if (0 <= new_row < height and 0 <= new_col < width and
                    (new_row, new_col) not in visited):

                    # Calculate movement cost
                    neighbor_cost = costmap[new_row, new_col]
                    move_cost = direction_costs[i] * (1 + neighbor_cost / 100.0)
                    total_cost = cost + move_cost

                    if neighbor_cost < 90:  # Not heavily occupied
                        if (new_row, new_col) not in [item[1:] for item in pq]:
                            heapq.heappush(pq, (total_cost, new_row, new_col))
                            predecessors[(new_row, new_col)] = (row, col)

        return None  # No path found

    def reconstruct_path(self, predecessors, start_idx, goal_idx):
        """Reconstruct path from predecessors dictionary"""
        path = [goal_idx]
        current = goal_idx

        while current != start_idx:
            if current not in predecessors:
                return None  # No path exists
            current = predecessors[current]
            path.append(current)

        path.reverse()
        return path

    def indices_to_path(self, indices):
        """Convert map indices to Path message"""
        path_msg = Path()
        path_msg.header.frame_id = 'map'
        path_msg.header.stamp = self.get_clock().now().to_msg()

        for row, col in indices:
            x, y = self.map_to_world(row, col)

            pose = PoseStamped()
            pose.header.frame_id = 'map'
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.position.z = 0.0

            # Simple orientation (pointing towards next point)
            path_msg.poses.append(pose)

        return path_msg
```

## Local Path Following

### Controller Implementation

```python
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry
from tf2_ros import TransformException
from tf2_ros import Buffer, TransformListener
import math

class LocalController(Node):
    def __init__(self):
        super().__init__('local_controller')

        # Subscribers
        self.odom_sub = self.create_subscription(
            Odometry, 'odom', self.odom_callback, 10)

        self.path_sub = self.create_subscription(
            Path, 'local_plan', self.path_callback, 10)

        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)

        # TF buffer for transforms
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Robot state
        self.current_pose = None
        self.current_twist = None
        self.current_path = []
        self.path_index = 0

        # Controller parameters
        self.lookahead_distance = 0.5  # meters
        self.linear_kp = 1.0
        self.angular_kp = 2.0
        self.max_linear_speed = 0.5
        self.max_angular_speed = 1.0

        # Timer for control loop
        self.control_timer = self.create_timer(0.05, self.control_loop)  # 20 Hz

    def odom_callback(self, msg):
        """Process odometry data"""
        self.current_pose = msg.pose.pose
        self.current_twist = msg.twist.twist

    def path_callback(self, msg):
        """Process new path"""
        self.current_path = msg.poses
        self.path_index = 0  # Reset path index

    def control_loop(self):
        """Main control loop"""
        if not self.current_pose or not self.current_path:
            return

        # Get current robot position
        robot_x = self.current_pose.position.x
        robot_y = self.current_pose.position.y

        # Find next waypoint using pure pursuit
        target_point = self.find_lookahead_point(robot_x, robot_y)

        if target_point:
            # Calculate control commands
            cmd_vel = self.pure_pursuit_control(robot_x, robot_y, target_point)

            # Publish command
            self.cmd_vel_pub.publish(cmd_vel)
        else:
            # Stop if no target point found (end of path)
            stop_cmd = Twist()
            self.cmd_vel_pub.publish(stop_cmd)

    def find_lookahead_point(self, robot_x, robot_y):
        """Find point on path at lookahead distance"""
        if len(self.current_path) <= self.path_index:
            return None

        # Look for point at lookahead distance
        for i in range(self.path_index, len(self.current_path)):
            pose = self.current_path[i].pose
            path_x = pose.position.x
            path_y = pose.position.y

            distance = math.sqrt((path_x - robot_x)**2 + (path_y - robot_y)**2)

            if distance >= self.lookahead_distance:
                self.path_index = i  # Update path index
                return (path_x, path_y)

        # If no point found at lookahead distance, return the last point
        if self.current_path:
            last_pose = self.current_path[-1].pose
            return (last_pose.position.x, last_pose.position.y)

        return None

    def pure_pursuit_control(self, robot_x, robot_y, target_point):
        """Pure pursuit path following algorithm"""
        target_x, target_y = target_point

        # Calculate errors
        dx = target_x - robot_x
        dy = target_y - robot_y

        # Calculate robot heading from orientation
        robot_yaw = self.get_robot_yaw()

        # Transform target point to robot frame
        local_x = dx * math.cos(robot_yaw) + dy * math.sin(robot_yaw)
        local_y = -dx * math.sin(robot_yaw) + dy * math.cos(robot_yaw)

        # Calculate curvature (steering angle)
        if local_x != 0:
            curvature = 2 * local_y / (local_x**2 + local_y**2)
        else:
            curvature = 0

        # Calculate linear velocity based on curvature (slow down when turning)
        linear_vel = self.max_linear_speed * (1 - abs(curvature) * 0.5)
        linear_vel = max(0.1, min(linear_vel, self.max_linear_speed))  # Clamp

        # Calculate angular velocity
        angular_vel = self.angular_kp * curvature
        angular_vel = max(-self.max_angular_speed, min(angular_vel, self.max_angular_speed))

        # Create command
        cmd_vel = Twist()
        cmd_vel.linear.x = linear_vel
        cmd_vel.angular.z = angular_vel

        return cmd_vel

    def get_robot_yaw(self):
        """Extract yaw from robot orientation quaternion"""
        if not self.current_pose:
            return 0.0

        orientation = self.current_pose.orientation
        siny_cosp = 2 * (orientation.w * orientation.z + orientation.x * orientation.y)
        cosy_cosp = 1 - 2 * (orientation.y * orientation.y + orientation.z * orientation.z)
        return math.atan2(siny_cosp, cosy_cosp)
```

## Navigation Actions and Services

### Navigation Action Client

```python
from rclpy.action import ActionClient
from rclpy.node import Node
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped
import time

class NavigationClient(Node):
    def __init__(self):
        super().__init__('navigation_client')

        # Create action client
        self._action_client = ActionClient(
            self,
            NavigateToPose,
            'navigate_to_pose')

    def send_goal(self, x, y, theta):
        """Send navigation goal to Navigation2"""
        # Wait for action server
        self._action_client.wait_for_server()

        # Create goal message
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = x
        goal_msg.pose.pose.position.y = y
        goal_msg.pose.pose.position.z = 0.0

        # Convert angle to quaternion
        goal_msg.pose.pose.orientation.z = math.sin(theta / 2.0)
        goal_msg.pose.pose.orientation.w = math.cos(theta / 2.0)

        # Send goal
        self.get_logger().info(f'Sending navigation goal to ({x}, {y}, {theta})')
        send_goal_future = self._action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback)

        # Add done callback
        send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        """Handle goal response"""
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected')
            return

        self.get_logger().info('Goal accepted')
        get_result_future = goal_handle.get_result_async()
        get_result_future.add_done_callback(self.get_result_callback)

    def feedback_callback(self, feedback_msg):
        """Handle feedback during navigation"""
        feedback = feedback_msg.feedback
        self.get_logger().info(f'Current pose: {feedback.current_pose.pose.position.x:.2f}, {feedback.current_pose.pose.position.y:.2f}')

    def get_result_callback(self, future):
        """Handle navigation result"""
        result = future.result().result
        self.get_logger().info(f'Navigation result: {result}')
```

## Obstacle Avoidance

### Local Planner with Obstacle Avoidance

```python
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

class ObstacleAvoidanceController(Node):
    def __init__(self):
        super().__init__('obstacle_avoidance_controller')

        # Subscribers
        self.scan_sub = self.create_subscription(
            LaserScan, 'scan', self.scan_callback, 10)
        self.cmd_sub = self.create_subscription(
            Twist, 'cmd_vel', self.cmd_callback, 10)

        # Publishers
        self.avoid_cmd_pub = self.create_publisher(Twist, 'cmd_vel_avoid', 10)

        # Internal state
        self.current_scan = None
        self.base_cmd = Twist()
        self.safety_distance = 0.5  # meters

    def scan_callback(self, msg):
        """Process laser scan data"""
        self.current_scan = msg

    def cmd_callback(self, msg):
        """Process base velocity commands"""
        self.base_cmd = msg

    def detect_obstacles(self):
        """Detect obstacles in the path"""
        if not self.current_scan:
            return None

        # Get ranges in front of the robot (±30 degrees)
        front_ranges = self.get_front_ranges()

        # Find minimum distance in front
        min_distance = min(front_ranges) if front_ranges else float('inf')

        return min_distance

    def get_front_ranges(self):
        """Get laser ranges in front of robot (+/- 30 degrees)"""
        if not self.current_scan:
            return []

        # Calculate indices for front sector
        angle_min = self.current_scan.angle_min
        angle_increment = self.current_scan.angle_increment
        angle_max = self.current_scan.angle_max

        # Front sector: -30 to +30 degrees
        start_angle = -math.pi / 6  # -30 degrees
        end_angle = math.pi / 6     # +30 degrees

        start_idx = int((start_angle - angle_min) / angle_increment)
        end_idx = int((end_angle - angle_min) / angle_increment)

        # Clamp indices
        start_idx = max(0, start_idx)
        end_idx = min(len(self.current_scan.ranges), end_idx)

        return self.current_scan.ranges[start_idx:end_idx]

    def obstacle_avoidance_control(self):
        """Apply obstacle avoidance to base commands"""
        min_distance = self.detect_obstacles()

        if min_distance is None:
            return self.base_cmd

        # If obstacle is too close, modify the command
        if min_distance < self.safety_distance:
            # Reduce forward speed proportionally to distance
            reduction_factor = min_distance / self.safety_distance
            avoid_cmd = Twist()
            avoid_cmd.linear.x = self.base_cmd.linear.x * reduction_factor
            avoid_cmd.angular.z = self.base_cmd.angular.z

            # If very close, add evasive maneuver
            if min_distance < self.safety_distance * 0.5:
                # Check which side has more space
                left_free = self.check_side_free('left')
                right_free = self.check_side_free('right')

                if left_free > right_free:
                    avoid_cmd.angular.z += 0.5  # Turn left
                else:
                    avoid_cmd.angular.z -= 0.5  # Turn right

            return avoid_cmd

        return self.base_cmd

    def check_side_free(self, side):
        """Check free space on left or right side"""
        if not self.current_scan:
            return 0

        ranges = self.current_scan.ranges
        mid_idx = len(ranges) // 2

        if side == 'left':
            # Check left side (first quarter of ranges)
            side_ranges = ranges[:mid_idx//2]
        else:  # right
            # Check right side (last quarter of ranges)
            side_ranges = ranges[mid_idx + mid_idx//2:]

        if side_ranges:
            return min([r for r in side_ranges if not math.isinf(r) and not math.isnan(r)] or [0])

        return 0
```

## Recovery Behaviors

### Recovery Actions

Navigation2 includes various recovery behaviors:

```python
from rclpy.action import ActionServer
from rclpy.node import Node
from nav2_msgs.action import BackUp, Spin, Wait
import time

class RecoveryActions(Node):
    def __init__(self):
        super().__init__('recovery_actions')

        # Recovery action servers
        self.backup_server = ActionServer(
            self,
            BackUp,
            'backup',
            self.backup_callback)

        self.spin_server = ActionServer(
            self,
            Spin,
            'spin',
            self.spin_callback)

        self.wait_server = ActionServer(
            self,
            Wait,
            'wait',
            self.wait_callback)

    def backup_callback(self, goal_handle):
        """Execute backup recovery behavior"""
        self.get_logger().info('Executing backup recovery')

        # Create velocity publisher
        cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)

        # Backup parameters
        backup_dist = goal_handle.request.target.x if hasattr(goal_handle.request.target, 'x') else 0.3
        backup_speed = 0.1  # m/s
        duration = backup_dist / backup_speed

        # Execute backup
        start_time = time.time()
        while time.time() - start_time < duration and not goal_handle.is_cancel_requested:
            cmd = Twist()
            cmd.linear.x = -backup_speed
            cmd_pub.publish(cmd)
            time.sleep(0.05)  # 20 Hz

        # Stop
        stop_cmd = Twist()
        cmd_pub.publish(stop_cmd)

        if goal_handle.is_cancel_requested:
            goal_handle.canceled()
            return BackUp.Result()

        goal_handle.succeed()
        return BackUp.Result()

    def spin_callback(self, goal_handle):
        """Execute spin recovery behavior"""
        self.get_logger().info('Executing spin recovery')

        cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)

        # Spin parameters
        angle_to_turn = goal_handle.request.target_theta if hasattr(goal_handle.request.target_theta, 'data') else 1.57  # 90 degrees
        spin_speed = 0.5  # rad/s
        duration = abs(angle_to_turn) / spin_speed

        start_time = time.time()
        direction = 1 if angle_to_turn > 0 else -1

        while time.time() - start_time < duration and not goal_handle.is_cancel_requested:
            cmd = Twist()
            cmd.angular.z = spin_speed * direction
            cmd_pub.publish(cmd)
            time.sleep(0.05)

        # Stop
        stop_cmd = Twist()
        cmd_pub.publish(stop_cmd)

        if goal_handle.is_cancel_requested:
            goal_handle.canceled()
            return Spin.Result()

        goal_handle.succeed()
        return Spin.Result()

    def wait_callback(self, goal_handle):
        """Execute wait recovery behavior"""
        self.get_logger().info('Executing wait recovery')

        wait_time = goal_handle.request.time.sec + goal_handle.request.time.nanosec * 1e-9

        start_time = time.time()
        while time.time() - start_time < wait_time and not goal_handle.is_cancel_requested:
            time.sleep(0.1)

        if goal_handle.is_cancel_requested:
            goal_handle.canceled()
            return Wait.Result()

        goal_handle.succeed()
        return Wait.Result()
```

## Learning Objectives

After completing this chapter, you will be able to:
- Configure and use Navigation2 for robot navigation
- Implement global path planning algorithms
- Develop local path following controllers
- Integrate obstacle avoidance with navigation
- Implement recovery behaviors for navigation failures

## Hands-on Exercise

Create a complete navigation system that includes global path planning, local path following, and obstacle avoidance. Test the system in simulation with various obstacle configurations.

:::tip
Use `rviz2` to visualize the navigation process, including the global plan, local plan, costmaps, and robot trajectory.
:::

:::note
Navigation2 performance depends heavily on accurate localization, so ensure your robot's localization system is properly configured before deploying navigation.
:::