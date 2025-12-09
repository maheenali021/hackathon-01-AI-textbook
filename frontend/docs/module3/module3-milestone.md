---
sidebar_position: 4
title: "Module 3 Milestone Project: Isaac ROS Perception Pipeline"
---

# Module 3 Milestone Project: Isaac ROS Perception Pipeline

## Project Overview

In this milestone project, you'll create a complete perception pipeline using Isaac ROS packages. You'll integrate multiple perception capabilities including visual SLAM, object detection, and sensor processing to enable a robot to understand and navigate its environment.

## Learning Objectives

After completing this project, you will be able to:
- Configure and run Isaac ROS perception nodes
- Integrate multiple perception capabilities in a single pipeline
- Process sensor data using GPU acceleration
- Combine perception outputs for navigation and manipulation
- Optimize perception pipelines for real-time performance

## Project Requirements

Create a perception pipeline with:
1. Visual SLAM for mapping and localization
2. Object detection for environment understanding
3. Depth estimation for obstacle detection
4. Sensor fusion for comprehensive perception
5. Integration with navigation systems

## Step-by-Step Implementation

### Step 1: Create the Perception Launch File

Create the file `perception_pipeline/launch/perception_pipeline.launch.py`:

```python
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Launch arguments
    use_composition = LaunchConfiguration('use_composition', default='False')
    use_viz = LaunchConfiguration('use_viz', default='True')
    use_laser = LaunchConfiguration('use_laser', default='True')
    use_pointcloud = LaunchConfiguration('use_pointcloud', default='True')

    # Package directories
    pkg_isaac_ros_visual_slam = get_package_share_directory('isaac_ros_visual_slam')
    pkg_isaac_ros_detect_and_track = get_package_share_directory('isaac_ros_detect_and_track')
    pkg_isaac_ros_stereo_image_proc = get_package_share_directory('isaac_ros_stereo_image_proc')
    pkg_isaac_ros_occupancy_grid_localizer = get_package_share_directory('isaac_ros_occupancy_grid_localizer')
    pkg_isaac_ros_ess = get_package_share_directory('isaac_ros_ess')
    pkg_isaac_ros_gxf = get_package_share_directory('isaac_ros_gxf')
    pkg_isaac_ros_apriltag = get_package_share_directory('isaac_ros_apriltag')

    # Visual SLAM node
    visual_slam_node = ComposableNode(
        name='visual_slam_node',
        package='isaac_ros_visual_slam',
        plugin='isaac_ros::slam::VisualSLAMNode',
        parameters=[{
            'enable_occupancy_grid': True,
            'enable_diagnostics': False,
            'occupancy_grid_resolution': 0.05,
            'min_num_features': 100,
            'enable_slam_visualization': True,
            'enable_landmarks_display': True,
        }],
        remappings=[
            ('stereo_camera/left/image', 'camera/left/image_raw'),
            ('stereo_camera/left/camera_info', 'camera/left/camera_info'),
            ('stereo_camera/right/image', 'camera/right/image_raw'),
            ('stereo_camera/right/camera_info', 'camera/right/camera_info'),
            ('visual_slam/imu', 'imu/data'),
        ]
    )

    # Isaac ROS Apriltag node for precise localization
    apriltag_node = ComposableNode(
        name='apriltag',
        package='isaac_ros_apriltag',
        plugin='nvidia::isaac_ros::apriltag::AprilTagNode',
        parameters=[{
            'size': 0.32,
            'max_tags': 64,
            'tile_size': 2,
            'quad_decimate': 2.0,
            'quad_sigma': 0.0,
            'refine_edges': True,
            'decode_sharpening': 0.25,
            'min_tag_width': 8,
        }],
        remappings=[
            ('image', 'camera/image_raw'),
            ('camera_info', 'camera/camera_info'),
            ('detections', 'apriltag_detections'),
        ]
    )

    # Stereo DNN node for object detection
    stereo_dnn_node = ComposableNode(
        name='stereo_dnn',
        package='isaac_ros_stereo_dnn',
        plugin='nvidia::isaac_ros::stereo_dnn::StereoDNNNode',
        parameters=[{
            'network_info.input_layer_names': ['input'],
            'network_info.output_layer_names': ['output'],
            'network_info.network_name': 'detectnet',
            'network_info.model_namespace': 'Isaac-ROS-Models',
            'input_layer_width': 960,
            'input_layer_height': 544,
            'max_batch_size': 1,
            'engine_file_path': '/tmp/detectnet.engine',
            'threshold': 0.5,
            'enable_padding': True,
            'input_tensor': 'input',
            'output_bbox_tensor': 'output',
            'output_covariance_tensor': 'output_cov',
            'bbox_container': 'bbox',
            'covariance_container': 'cov',
            'image_width': 960,
            'image_height': 576,
            'do_resize': True,
            'scale': 1.0,
            'flip_image': 0,
            'encoding_desired': 'rgb8',
            'apply_edge_aware_threshold': False,
            'enable_segmentation_masks': False,
            'mask_threshold': 0.5,
            'mask_bbox_overlap_threshold': 0.8,
            'enable_bounding_box_crop': False,
            'class_labels_file_path': '/tmp/class_labels.txt',
        }],
        remappings=[
            ('left_image', 'camera/left/image_raw'),
            ('right_image', 'camera/right/image_raw'),
            ('left_camera_info', 'camera/left/camera_info'),
            ('right_camera_info', 'camera/right/camera_info'),
            ('detections', 'stereo_detections'),
        ]
    )

    # Point Cloud Node for 3D reconstruction
    point_cloud_node = ComposableNode(
        name='point_cloud',
        package='isaac_ros_stereo_image_proc',
        plugin='nvidia::isaac_ros::stereo_image_proc::PointCloudNode',
        parameters=[{
            'use_color': True,
            'unit_scaling': 1.0,
        }],
        remappings=[
            ('left/image_rect_color', 'camera/left/image_rect_color'),
            ('left/camera_info', 'camera/left/camera_info'),
            ('right/camera_info', 'camera/right/camera_info'),
            ('disparity', 'stereo/disparity'),
            ('points', 'points'),
        ]
    )

    # Container for all perception nodes
    perception_container = ComposableNodeContainer(
        name='perception_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container_mt',
        composable_node_descriptions=[
            visual_slam_node,
            apriltag_node,
            stereo_dnn_node,
            point_cloud_node,
        ],
        output='screen'
    )

    # RViz2 node for visualization
    rviz_config = os.path.join(
        pkg_isaac_ros_visual_slam,
        'rviz',
        'visual_slam.rviz'
    )

    rviz_node = Node(
        condition=IfCondition(use_viz),
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        output='screen'
    )

    # Occupancy grid localizer node
    occupancy_grid_localizer_node = ComposableNode(
        name='occupancy_grid_localizer',
        package='isaac_ros_occupancy_grid_localizer',
        plugin='nvidia::isaac_ros::occupancy_grid_localizer::OccupancyGridLocalizerNode',
        parameters=[{
            'map_width': 50.0,
            'map_height': 50.0,
            'map_resolution': 0.05,
            'map_frame': 'map',
            'robot_frame': 'base_link',
            'update_mode': 'match_global_localization',
            'global_localization_interval': 5.0,
        }],
        remappings=[
            ('localization', 'localization'),
            ('occupancy_grid', 'map'),
            ('global_localization_pose', 'initialpose'),
        ]
    )

    # Container for localization nodes
    localization_container = ComposableNodeContainer(
        name='localization_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container_mt',
        composable_node_descriptions=[
            occupancy_grid_localizer_node,
        ],
        output='screen'
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_composition', default_value='False',
            description='Use composed bringup if True'),
        DeclareLaunchArgument(
            'use_viz', default_value='True',
            description='Launch RViz if True'),
        DeclareLaunchArgument(
            'use_laser', default_value='True',
            description='Use laser-based perception if True'),
        DeclareLaunchArgument(
            'use_pointcloud', default_value='True',
            description='Use point cloud processing if True'),
        perception_container,
        localization_container,
        rviz_node,
    ])
```

### Step 2: Create the Navigation Integration Node

Create the file `perception_pipeline/src/navigation_integration.py`:

```python
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import LaserScan, PointCloud2
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import OccupancyGrid, Odometry
from visualization_msgs.msg import MarkerArray
from builtin_interfaces.msg import Duration
from std_msgs.msg import Header
import numpy as np
import math
from tf2_ros import TransformListener, Buffer
import tf2_geometry_msgs
from geometry_msgs.msg import Point, Pose, PoseStamped


class NavigationIntegration(Node):
    def __init__(self):
        super().__init__('navigation_integration')

        # QoS profile for sensor data
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # Subscriptions
        self.scan_sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            qos_profile
        )

        self.map_sub = self.create_subscription(
            OccupancyGrid,
            '/map',
            self.map_callback,
            10
        )

        self.odom_sub = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )

        self.detection_sub = self.create_subscription(
            MarkerArray,
            '/object_detections',
            self.detection_callback,
            10
        )

        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.goal_pub = self.create_publisher(PoseStamped, '/goal_pose', 10)
        self.safe_path_pub = self.create_publisher(PoseStamped, '/safe_path', 10)

        # TF buffer and listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Robot state
        self.current_pose = Pose()
        self.current_scan = None
        self.current_map = None
        self.obstacles = []
        self.navigation_goals = []

        # Navigation parameters
        self.safe_distance = 0.5  # meters
        self.max_linear_speed = 0.5
        self.max_angular_speed = 1.0

        # Timer for navigation loop
        self.nav_timer = self.create_timer(0.1, self.navigation_loop)

    def scan_callback(self, msg):
        """Process LIDAR scan data"""
        self.current_scan = msg
        self.get_logger().info(f'Received scan with {len(msg.ranges)} points')

    def map_callback(self, msg):
        """Process occupancy grid map"""
        self.current_map = msg
        self.get_logger().info('Received occupancy grid map')

    def odom_callback(self, msg):
        """Process odometry data"""
        self.current_pose = msg.pose.pose

    def detection_callback(self, msg):
        """Process object detection results"""
        self.obstacles = []
        for marker in msg.markers:
            if marker.type == marker.CUBE:  # Assuming detected objects are cubes
                obstacle = {
                    'position': marker.pose.position,
                    'size': marker.scale,
                    'label': marker.ns
                }
                self.obstacles.append(obstacle)

    def navigation_loop(self):
        """Main navigation decision-making loop"""
        if not self.current_scan or not self.current_pose:
            return

        # Calculate safe movement based on sensor data
        cmd_vel = Twist()

        # Check for obstacles in front of robot
        front_ranges = self.get_front_scan_ranges()
        if front_ranges:
            min_distance = min(front_ranges)
            if min_distance < self.safe_distance:
                # Obstacle too close - stop or turn
                cmd_vel.linear.x = 0.0
                cmd_vel.angular.z = 0.5  # Turn right
            else:
                # Path clear - move forward
                cmd_vel.linear.x = min(self.max_linear_speed, min_distance * 0.5)
                cmd_vel.angular.z = 0.0

        # Apply obstacle avoidance from detections
        if self.obstacles:
            avoidance_cmd = self.calculate_obstacle_avoidance()
            cmd_vel.linear.x = min(cmd_vel.linear.x, avoidance_cmd.linear.x)
            cmd_vel.angular.z += avoidance_cmd.angular.z

        # Publish navigation command
        self.cmd_vel_pub.publish(cmd_vel)

        # Log navigation decision
        self.get_logger().info(
            f'Navigation: Lin: {cmd_vel.linear.x:.2f}, '
            f'Ang: {cmd_vel.angular.z:.2f}, '
            f'Min dist: {min(front_ranges) if front_ranges else 0:.2f}m'
        )

    def get_front_scan_ranges(self):
        """Get scan ranges in front of the robot (forward 90 degrees)"""
        if not self.current_scan:
            return []

        # Calculate indices for front 90 degrees
        angle_min = self.current_scan.angle_min
        angle_increment = self.current_scan.angle_increment
        ranges = self.current_scan.ranges

        # Front angles: -45 to +45 degrees
        start_idx = int((math.radians(-45) - angle_min) / angle_increment)
        end_idx = int((math.radians(45) - angle_min) / angle_increment)

        start_idx = max(0, start_idx)
        end_idx = min(len(ranges), end_idx)

        return ranges[start_idx:end_idx]

    def calculate_obstacle_avoidance(self):
        """Calculate avoidance commands based on object detections"""
        cmd_vel = Twist()

        # Find closest obstacle in front of robot
        closest_obstacle = None
        min_distance = float('inf')

        for obstacle in self.obstacles:
            # Calculate distance to obstacle
            dx = obstacle['position'].x - self.current_pose.position.x
            dy = obstacle['position'].y - self.current_pose.position.y
            distance = math.sqrt(dx*dx + dy*dy)

            if distance < min_distance and distance < 2.0:  # Only consider obstacles within 2m
                min_distance = distance
                closest_obstacle = obstacle

        if closest_obstacle and min_distance < 1.0:
            # Calculate direction to move away from obstacle
            dx = closest_obstacle['position'].x - self.current_pose.position.x
            dy = closest_obstacle['position'].y - self.current_pose.position.y

            # Calculate angle to obstacle
            obstacle_angle = math.atan2(dy, dx)
            robot_yaw = self.get_robot_yaw()

            # Calculate relative angle
            relative_angle = obstacle_angle - robot_yaw
            relative_angle = math.atan2(math.sin(relative_angle), math.cos(relative_angle))

            # Turn away from obstacle
            if abs(relative_angle) < math.pi / 2:
                # Obstacle is in front - turn away
                cmd_vel.angular.z = 0.5 if relative_angle > 0 else -0.5
                cmd_vel.linear.x = max(0.1, self.max_linear_speed * (1.0 - min_distance/1.0))

        return cmd_vel

    def get_robot_yaw(self):
        """Extract yaw from robot's orientation quaternion"""
        orientation = self.current_pose.orientation
        siny_cosp = 2 * (orientation.w * orientation.z + orientation.x * orientation.y)
        cosy_cosp = 1 - 2 * (orientation.y * orientation.y + orientation.z * orientation.z)
        return math.atan2(siny_cosp, cosy_cosp)

    def find_navigation_goals(self):
        """Find potential navigation goals based on map and detections"""
        goals = []

        if self.current_map and self.obstacles:
            # Look for areas that are free in the map but may contain interesting objects
            for obstacle in self.obstacles:
                # Create a goal near interesting objects
                goal = PoseStamped()
                goal.header.stamp = self.get_clock().now().to_msg()
                goal.header.frame_id = 'map'
                goal.pose.position.x = obstacle['position'].x
                goal.pose.position.y = obstacle['position'].y
                goal.pose.position.z = 0.0
                goal.pose.orientation.w = 1.0

                goals.append(goal)

        return goals


def main(args=None):
    rclpy.init(args=args)
    navigation_integration = NavigationIntegration()

    try:
        rclpy.spin(navigation_integration)
    except KeyboardInterrupt:
        pass
    finally:
        navigation_integration.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

### Step 3: Create the Perception Processing Node

Create the file `perception_pipeline/src/perception_processor.py`:

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo, PointCloud2
from vision_msgs.msg import Detection2DArray
from geometry_msgs.msg import Point, Pose, TransformStamped
from tf2_ros import TransformBroadcaster
from builtin_interfaces.msg import Time
from std_msgs.msg import Header
import numpy as np
import cv2
from cv_bridge import CvBridge
import message_filters
from tf2_ros import Buffer, TransformListener
import tf2_geometry_msgs
from visualization_msgs.msg import MarkerArray, Marker
from geometry_msgs.msg import Point as GeometryPoint


class PerceptionProcessor(Node):
    def __init__(self):
        super().__init__('perception_processor')

        # CV Bridge for image processing
        self.bridge = CvBridge()

        # Transform broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)

        # Publishers
        self.object_detection_pub = self.create_publisher(Detection2DArray, '/processed_detections', 10)
        self.marker_pub = self.create_publisher(MarkerArray, '/object_markers', 10)
        self.point_cloud_pub = self.create_publisher(PointCloud2, '/processed_pointcloud', 10)

        # Subscriptions with message filters for synchronization
        self.image_sub = message_filters.Subscriber(self, Image, '/camera/image_raw')
        self.camera_info_sub = message_filters.Subscriber(self, CameraInfo, '/camera/camera_info')

        # Approximate time synchronizer
        self.sync = message_filters.ApproximateTimeSynchronizer(
            [self.image_sub, self.camera_info_sub],
            queue_size=10,
            slop=0.1
        )
        self.sync.registerCallback(self.process_camera_data)

        # Internal state
        self.latest_image = None
        self.latest_camera_info = None

        # Object detection parameters
        self.confidence_threshold = 0.5
        self.class_labels = [
            'person', 'bicycle', 'car', 'motorcycle', 'airplane', 'bus', 'train',
            'truck', 'boat', 'traffic light', 'fire hydrant', 'stop sign',
            'parking meter', 'bench', 'bird', 'cat', 'dog', 'horse', 'sheep',
            'cow', 'elephant', 'bear', 'zebra', 'giraffe', 'backpack', 'umbrella',
            'handbag', 'tie', 'suitcase', 'frisbee', 'skis', 'snowboard',
            'sports ball', 'kite', 'baseball bat', 'baseball glove', 'skateboard',
            'surfboard', 'tennis racket', 'bottle', 'wine glass', 'cup', 'fork',
            'knife', 'spoon', 'bowl', 'banana', 'apple', 'sandwich', 'orange',
            'broccoli', 'carrot', 'hot dog', 'pizza', 'donut', 'cake', 'chair',
            'couch', 'potted plant', 'bed', 'dining table', 'toilet', 'tv',
            'laptop', 'mouse', 'remote', 'keyboard', 'cell phone', 'microwave',
            'oven', 'toaster', 'sink', 'refrigerator', 'book', 'clock', 'vase',
            'scissors', 'teddy bear', 'hair drier', 'toothbrush'
        ]

    def process_camera_data(self, image_msg, camera_info_msg):
        """Process synchronized camera data"""
        try:
            # Convert ROS image to OpenCV
            cv_image = self.bridge.imgmsg_to_cv2(image_msg, desired_encoding='bgr8')

            # Process the image for object detection
            detections = self.detect_objects(cv_image)

            # Create detection message
            detection_msg = self.create_detection_message(detections, image_msg.header)

            # Publish detections
            self.object_detection_pub.publish(detection_msg)

            # Create and publish visualization markers
            marker_array = self.create_visualization_markers(detections, image_msg.header)
            self.marker_pub.publish(marker_array)

            # Log detection results
            self.get_logger().info(f'Detected {len(detections)} objects')

        except Exception as e:
            self.get_logger().error(f'Error processing camera data: {str(e)}')

    def detect_objects(self, cv_image):
        """Simulate object detection (in real implementation, this would use a DNN)"""
        # In a real implementation, this would use Isaac ROS DNN packages
        # For simulation, we'll create some synthetic detections
        height, width = cv_image.shape[:2]

        # Simulated detections (in practice, these would come from a neural network)
        detections = []

        # Add some synthetic detections for demonstration
        if np.random.random() > 0.7:  # 30% chance of detection
            for i in range(np.random.randint(1, 4)):  # 1-3 detections
                x = np.random.randint(0, width - 100)
                y = np.random.randint(0, height - 100)
                w = np.random.randint(50, 150)
                h = np.random.randint(50, 150)

                confidence = np.random.uniform(0.6, 0.99)
                class_id = np.random.randint(0, len(self.class_labels))
                class_name = self.class_labels[class_id]

                detection = {
                    'bbox': (x, y, w, h),
                    'confidence': confidence,
                    'class_id': class_id,
                    'class_name': class_name
                }
                detections.append(detection)

        return detections

    def create_detection_message(self, detections, header):
        """Create vision_msgs/Detection2DArray message"""
        detection_array = Detection2DArray()
        detection_array.header = header

        for detection in detections:
            bbox_x, bbox_y, bbox_w, bbox_h = detection['bbox']

            detection_2d = Detection2D()
            detection_2d.header = header

            # Set the bounding box
            detection_2d.bbox.center.x = bbox_x + bbox_w / 2
            detection_2d.bbox.center.y = bbox_y + bbox_h / 2
            detection_2d.bbox.size_x = bbox_w
            detection_2d.bbox.size_y = bbox_h

            # Set the confidence
            detection_2d.results.append(Detection2D())
            detection_2d.results[0].score = detection['confidence']
            detection_2d.results[0].class_id = detection['class_name']

            detection_array.detections.append(detection_2d)

        return detection_array

    def create_visualization_markers(self, detections, header):
        """Create visualization markers for detected objects"""
        marker_array = MarkerArray()

        for i, detection in enumerate(detections):
            # Create a marker for the bounding box
            marker = Marker()
            marker.header = header
            marker.ns = "detections"
            marker.id = i
            marker.type = Marker.CUBE
            marker.action = Marker.ADD

            bbox_x, bbox_y, bbox_w, bbox_h = detection['bbox']

            # Position in 3D space (projected from 2D image)
            marker.pose.position.x = (bbox_x + bbox_w / 2) / header.frame_id.width * 2 - 1  # Normalize to -1 to 1
            marker.pose.position.y = (bbox_y + bbox_h / 2) / header.frame_id.height * 2 - 1  # Normalize to -1 to 1
            marker.pose.position.z = 1.0  # Fixed distance for visualization

            # Size based on bounding box
            marker.scale.x = bbox_w / header.frame_id.width * 2
            marker.scale.y = bbox_h / header.frame_id.height * 2
            marker.scale.z = 0.1  # Thin box for visualization

            # Color based on class
            marker.color.r = 1.0
            marker.color.g = 0.0 if detection['confidence'] > 0.7 else 0.5
            marker.color.b = 0.0 if detection['confidence'] > 0.7 else 0.5
            marker.color.a = detection['confidence']

            marker_array.markers.append(marker)

            # Create a text marker for the label
            text_marker = Marker()
            text_marker.header = header
            text_marker.ns = "labels"
            text_marker.id = i + 1000  # Different ID space
            text_marker.type = Marker.TEXT_VIEW_FACING
            text_marker.action = Marker.ADD

            text_marker.pose.position.x = marker.pose.position.x
            text_marker.pose.position.y = marker.pose.position.y - 0.1  # Below the box
            text_marker.pose.position.z = marker.pose.position.z

            text_marker.scale.z = 0.1  # Text size
            text_marker.color.r = 1.0
            text_marker.color.g = 1.0
            text_marker.color.b = 1.0
            text_marker.color.a = 1.0

            text_marker.text = f"{detection['class_name']}: {detection['confidence']:.2f}"

            marker_array.markers.append(text_marker)

        return marker_array


def main(args=None):
    rclpy.init(args=args)
    perception_processor = PerceptionProcessor()

    try:
        rclpy.spin(perception_processor)
    except KeyboardInterrupt:
        pass
    finally:
        perception_processor.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

### Step 4: Create the Main Pipeline Launch File

Create the file `perception_pipeline/launch/main_pipeline.launch.py`:

```python
import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Launch configurations
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    camera_namespace = LaunchConfiguration('camera_namespace', default='/camera')
    robot_namespace = LaunchConfiguration('robot_namespace', default='')

    # Package directories
    pkg_perception_pipeline = get_package_share_directory('perception_pipeline')
    pkg_isaac_ros_visual_slam = get_package_share_directory('isaac_ros_visual_slam')
    pkg_isaac_ros_detect_and_track = get_package_share_directory('isaac_ros_detect_and_track')

    # Include the perception pipeline launch
    perception_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_perception_pipeline, 'launch', 'perception_pipeline.launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time
        }.items()
    )

    # Perception processing node
    perception_processor_node = Node(
        package='perception_pipeline',
        executable='perception_processor',
        name='perception_processor',
        parameters=[{
            'use_sim_time': use_sim_time,
        }],
        remappings=[
            ('/camera/image_raw', [camera_namespace, '/image_raw']),
            ('/camera/camera_info', [camera_namespace, '/camera_info']),
        ],
        output='screen'
    )

    # Navigation integration node
    navigation_integration_node = Node(
        package='perception_pipeline',
        executable='navigation_integration',
        name='navigation_integration',
        parameters=[{
            'use_sim_time': use_sim_time,
        }],
        output='screen'
    )

    # Performance monitor node
    performance_monitor_node = Node(
        package='perception_pipeline',
        executable='performance_monitor',
        name='performance_monitor',
        parameters=[{
            'use_sim_time': use_sim_time,
        }],
        output='screen'
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation time if true'),
        DeclareLaunchArgument(
            'camera_namespace',
            default_value='/camera',
            description='Namespace for camera topics'),
        DeclareLaunchArgument(
            'robot_namespace',
            default_value='',
            description='Namespace for robot topics'),

        perception_launch,
        perception_processor_node,
        navigation_integration_node,
        performance_monitor_node,
    ])
```

### Step 5: Package Configuration

Create the file `perception_pipeline/package.xml`:

```xml
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>perception_pipeline</name>
  <version>0.0.0</version>
  <description>Isaac ROS perception pipeline for Module 3 milestone project</description>
  <maintainer email="your_email@example.com">Your Name</maintainer>
  <license>TODO: License declaration</license>

  <buildtool_depend>ament_cmake</buildtool_depend>
  <buildtool_depend>ament_cmake_python</buildtool_depend>

  <depend>rclcpp</depend>
  <depend>rclpy</depend>
  <depend>std_msgs</depend>
  <depend>sensor_msgs</depend>
  <depend>geometry_msgs</depend>
  <depend>nav_msgs</depend>
  <depend>visualization_msgs</depend>
  <depend>vision_msgs</depend>
  <depend>tf2</depend>
  <depend>tf2_ros</depend>
  <depend>tf2_geometry_msgs</depend>
  <depend>cv_bridge</depend>
  <depend>message_filters</depend>

  <depend>isaac_ros_visual_slam</depend>
  <depend>isaac_ros_detect_and_track</depend>
  <depend>isaac_ros_stereo_image_proc</depend>
  <depend>isaac_ros_occupancy_grid_localizer</depend>
  <depend>isaac_ros_apriltag</depend>

  <exec_depend>launch</exec_depend>
  <exec_depend>launch_ros</exec_depend>

  <test_depend>ament_lint_auto</test_depend>
  <test_depend>ament_lint_common</test_depend>

  <export>
    <build_type>ament_cmake</build_type>
  </export>
</package>
```

And create the `perception_pipeline/CMakeLists.txt`:

```cmake
cmake_minimum_required(VERSION 3.8)
project(perception_pipeline)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(vision_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(message_filters REQUIRED)

# Install Python modules
ament_python_install_package(${PROJECT_NAME})

# Install Python executables
install(PROGRAMS
  src/navigation_integration.py
  src/perception_processor.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)

# Install config files
install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}/
)

# Install RViz config
install(DIRECTORY rviz
  DESTINATION share/${PROJECT_NAME}/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
```

## Running the Perception Pipeline

### To run the complete perception pipeline:

```bash
# Terminal 1: Start the perception pipeline
ros2 launch perception_pipeline main_pipeline.launch.py

# Terminal 2: Simulate camera and sensor data (if using real robot simulation)
# This would typically come from your robot's sensors

# Terminal 3: Monitor perception outputs
ros2 topic echo /processed_detections
ros2 topic echo /object_markers
ros2 topic echo /map
```

## Testing Your Implementation

### Test 1: Verify Perception Nodes Are Running

```bash
# Check active nodes
ros2 node list | grep perception

# Check topics
ros2 topic list | grep -E "(detection|scan|map|camera)"
```

### Test 2: Monitor Performance

```bash
# Monitor detection rates
ros2 topic echo /processed_detections --field detections | head -n 10

# Check navigation commands
ros2 topic echo /cmd_vel | head -n 5
```

### Test 3: Visualization

```bash
# Launch RViz to visualize the perception outputs
ros2 run rviz2 rviz2 -d `ros2 pkg prefix perception_pipeline`/share/perception_pipeline/rviz/config.rviz
```

## Enhancement Challenges

1. Add more perception capabilities (semantic segmentation, instance segmentation)
2. Implement a learning-based perception system
3. Add sensor fusion between different modalities
4. Optimize the pipeline for edge devices
5. Add robustness to lighting and environmental changes

## Solution Summary

This milestone project demonstrates how to create a complete perception pipeline using Isaac ROS with:
- Visual SLAM for mapping and localization
- Object detection for environment understanding
- Sensor fusion for comprehensive perception
- Navigation integration for autonomous behavior
- Performance monitoring for optimization

## Learning Review

After completing this project, reflect on:
- How did GPU acceleration improve the perception pipeline performance?
- What challenges did you face when integrating multiple perception capabilities?
- How did sensor fusion improve the robot's understanding of its environment?
- What trade-offs exist between accuracy and real-time performance?

## Next Steps

This perception pipeline provides the foundation for the AI and decision-making systems you'll develop in Module 4. You can use this pipeline to:
- Feed perception data to AI reasoning systems
- Create semantic maps for higher-level planning
- Generate training data for machine learning models
- Test human-robot interaction scenarios