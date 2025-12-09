---
sidebar_position: 4
title: "Chapter 4: Sensor Simulation in Digital Twins"
---

# Sensor Simulation in Digital Twins

## Introduction to Sensor Simulation

Sensor simulation is a crucial aspect of digital twin technology in robotics. It allows for testing perception algorithms, sensor fusion, and robot behaviors in a safe, controlled virtual environment before deployment on real hardware.

## Types of Sensors in Robotics

### Camera Sensors

Camera sensors provide visual information to robots. In simulation, they can be configured with various properties:

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
import numpy as np
import cv2

class CameraSimulator(Node):
    def __init__(self):
        super().__init__('camera_simulator')

        # Publisher for camera images
        self.image_pub = self.create_publisher(Image, '/camera/image_raw', 10)
        self.info_pub = self.create_publisher(CameraInfo, '/camera/camera_info', 10)

        # CV Bridge for image conversion
        self.bridge = CvBridge()

        # Camera parameters
        self.width = 640
        self.height = 480
        self.fps = 30

        # Timer for image generation
        self.timer = self.create_timer(1.0/self.fps, self.generate_image)

        self.get_logger().info('Camera simulator initialized')

    def generate_image(self):
        """Generate simulated camera image"""
        # Create synthetic image (in real implementation, this would come from simulation engine)
        image = np.zeros((self.height, self.width, 3), dtype=np.uint8)

        # Add some synthetic objects to the image
        cv2.circle(image, (self.width//2, self.height//2), 50, (255, 0, 0), -1)  # Blue circle
        cv2.rectangle(image, (100, 100), (200, 200), (0, 255, 0), 2)  # Green rectangle

        # Add noise to make it more realistic
        noise = np.random.normal(0, 10, image.shape).astype(np.uint8)
        image = cv2.add(image, noise)

        # Convert to ROS Image message
        ros_image = self.bridge.cv2_to_imgmsg(image, encoding='bgr8')
        ros_image.header.stamp = self.get_clock().now().to_msg()
        ros_image.header.frame_id = 'camera_link'

        # Publish image
        self.image_pub.publish(ros_image)

        # Publish camera info
        self.publish_camera_info()

    def publish_camera_info(self):
        """Publish camera calibration information"""
        info_msg = CameraInfo()
        info_msg.header.stamp = self.get_clock().now().to_msg()
        info_msg.header.frame_id = 'camera_link'
        info_msg.width = self.width
        info_msg.height = self.height

        # Camera intrinsic parameters (example values)
        info_msg.k = [500.0, 0.0, self.width/2,    # fx, 0, cx
                      0.0, 500.0, self.height/2,  # 0, fy, cy
                      0.0, 0.0, 1.0]              # 0, 0, 1

        info_msg.d = [0.0, 0.0, 0.0, 0.0, 0.0]  # Distortion coefficients

        self.info_pub.publish(info_msg)
```

### LIDAR Sensors

LIDAR sensors provide distance measurements in a 2D or 3D space:

```python
from sensor_msgs.msg import LaserScan
import math

class LIDARSimulator(Node):
    def __init__(self):
        super().__init__('lidar_simulator')

        # Publisher for LIDAR scan
        self.scan_pub = self.create_publisher(LaserScan, '/scan', 10)

        # LIDAR parameters
        self.angle_min = -math.pi
        self.angle_max = math.pi
        self.angle_increment = math.pi / 180  # 1 degree resolution
        self.time_increment = 0.0
        self.scan_time = 0.1
        self.range_min = 0.1
        self.range_max = 10.0

        # Timer for scan generation
        self.timer = self.create_timer(0.1, self.generate_scan)

    def generate_scan(self):
        """Generate simulated LIDAR scan"""
        num_scans = int((self.angle_max - self.angle_min) / self.angle_increment)

        # Generate ranges with simulated environment
        ranges = []
        for i in range(num_scans):
            angle = self.angle_min + i * self.angle_increment

            # Simulate environment with walls and obstacles
            distance = self.simulate_distance(angle)

            # Add noise to make it realistic
            noise = np.random.normal(0, 0.02)  # 2cm noise
            distance_with_noise = max(self.range_min, min(self.range_max, distance + noise))

            ranges.append(distance_with_noise)

        # Create LaserScan message
        scan_msg = LaserScan()
        scan_msg.header.stamp = self.get_clock().now().to_msg()
        scan_msg.header.frame_id = 'laser_frame'
        scan_msg.angle_min = self.angle_min
        scan_msg.angle_max = self.angle_max
        scan_msg.angle_increment = self.angle_increment
        scan_msg.time_increment = self.time_increment
        scan_msg.scan_time = self.scan_time
        scan_msg.range_min = self.range_min
        scan_msg.range_max = self.range_max
        scan_msg.ranges = ranges
        scan_msg.intensities = []  # Intensities are optional

        # Publish scan
        self.scan_pub.publish(scan_msg)

    def simulate_distance(self, angle):
        """Simulate distance measurement based on angle"""
        # Example: simple square room with side length 8m
        # Robot is in the center
        half_room_size = 4.0

        # Calculate distance to walls based on angle
        abs_angle = abs(angle) % (math.pi / 2)

        if abs_angle < math.pi / 4:
            # Horizontal wall
            distance = half_room_size / math.cos(abs_angle)
        else:
            # Vertical wall
            distance = half_room_size / math.sin(abs_angle)

        # Add some obstacles
        if 0.5 < distance < 2.0:
            # Add a column in front of the robot
            distance = 1.5

        return min(distance, self.range_max)
```

### IMU Sensors

IMU sensors provide information about acceleration, angular velocity, and orientation:

```python
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Vector3, Quaternion

class IMUSimulator(Node):
    def __init__(self):
        super().__init__('imu_simulator')

        # Publisher for IMU data
        self.imu_pub = self.create_publisher(Imu, '/imu/data', 10)

        # IMU parameters
        self.linear_acceleration_variance = 0.01
        self.angular_velocity_variance = 0.01
        self.orientation_variance = 0.001

        # Robot state (for simulating motion)
        self.position = [0.0, 0.0, 0.0]
        self.velocity = [0.0, 0.0, 0.0]
        self.acceleration = [0.0, 0.0, 0.0]

        # Timer for IMU data generation
        self.timer = self.create_timer(0.01, self.generate_imu_data)  # 100Hz

    def generate_imu_data(self):
        """Generate simulated IMU data"""
        imu_msg = Imu()
        imu_msg.header.stamp = self.get_clock().now().to_msg()
        imu_msg.header.frame_id = 'imu_link'

        # Simulate gravity in z-axis
        imu_msg.linear_acceleration.x = self.acceleration[0] + np.random.normal(0, self.linear_acceleration_variance)
        imu_msg.linear_acceleration.y = self.acceleration[1] + np.random.normal(0, self.linear_acceleration_variance)
        imu_msg.linear_acceleration.z = self.acceleration[2] + 9.81 + np.random.normal(0, self.linear_acceleration_variance)

        # Simulate angular velocity (could be from robot motion)
        imu_msg.angular_velocity.x = np.random.normal(0, self.angular_velocity_variance)
        imu_msg.angular_velocity.y = np.random.normal(0, self.angular_velocity_variance)
        imu_msg.angular_velocity.z = np.random.normal(0, self.angular_velocity_variance)

        # Simulate orientation (assuming robot is upright)
        imu_msg.orientation.x = np.random.normal(0, self.orientation_variance)
        imu_msg.orientation.y = np.random.normal(0, self.orientation_variance)
        imu_msg.orientation.z = np.random.normal(0, self.orientation_variance)
        imu_msg.orientation.w = 1.0  # Normalize quaternion

        # Set covariance matrices
        # Linear acceleration covariance
        imu_msg.linear_acceleration_covariance = [
            self.linear_acceleration_variance, 0, 0,
            0, self.linear_acceleration_variance, 0,
            0, 0, self.linear_acceleration_variance
        ]

        # Angular velocity covariance
        imu_msg.angular_velocity_covariance = [
            self.angular_velocity_variance, 0, 0,
            0, self.angular_velocity_variance, 0,
            0, 0, self.angular_velocity_variance
        ]

        # Orientation covariance
        imu_msg.orientation_covariance = [
            self.orientation_variance, 0, 0,
            0, self.orientation_variance, 0,
            0, 0, self.orientation_variance
        ]

        # Publish IMU data
        self.imu_pub.publish(imu_msg)
```

## Sensor Fusion Simulation

### Combining Multiple Sensors

```python
from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import PoseWithCovarianceStamped
import tf2_ros
from tf2_ros import TransformBroadcaster

class SensorFusionSimulator(Node):
    def __init__(self):
        super().__init__('sensor_fusion_simulator')

        # Subscribers for different sensors
        self.image_sub = self.create_subscription(
            Image, '/camera/image_raw', self.image_callback, 10)
        self.scan_sub = self.create_subscription(
            LaserScan, '/scan', self.scan_callback, 10)
        self.imu_sub = self.create_subscription(
            Imu, '/imu/data', self.imu_callback, 10)

        # Publishers for fused data
        self.fused_pub = self.create_publisher(
            PointCloud2, '/fused_pointcloud', 10)
        self.pose_pub = self.create_publisher(
            PoseWithCovarianceStamped, '/estimated_pose', 10)

        # TF broadcaster for transforms
        self.tf_broadcaster = TransformBroadcaster(self)

        # Internal state
        self.last_image = None
        self.last_scan = None
        self.last_imu = None

        # Timer for fusion
        self.fusion_timer = self.create_timer(0.1, self.perform_fusion)

    def image_callback(self, msg):
        """Handle image data"""
        self.last_image = msg

    def scan_callback(self, msg):
        """Handle LIDAR scan data"""
        self.last_scan = msg

    def imu_callback(self, msg):
        """Handle IMU data"""
        self.last_imu = msg

    def perform_fusion(self):
        """Perform sensor fusion to estimate robot state"""
        if not all([self.last_image, self.last_scan, self.last_imu]):
            return  # Not all sensors have published yet

        # Example fusion: combine IMU orientation with position from other sources
        estimated_pose = self.estimate_pose_from_sensors()

        # Publish fused estimate
        pose_msg = PoseWithCovarianceStamped()
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        pose_msg.header.frame_id = 'map'
        pose_msg.pose = estimated_pose

        self.pose_pub.publish(pose_msg)

        # Create fused point cloud from camera and LIDAR
        fused_cloud = self.create_fused_pointcloud()
        if fused_cloud:
            self.fused_pub.publish(fused_cloud)

    def estimate_pose_from_sensors(self):
        """Estimate pose by combining sensor data"""
        # This is a simplified example
        # In practice, you would use Kalman filters or particle filters
        from geometry_msgs.msg import PoseWithCovariance

        pose = PoseWithCovariance()

        # Use IMU for orientation
        pose.pose.orientation = self.last_imu.orientation

        # For position, you might combine odometry with sensor data
        # For simulation, we'll just use a fixed position with IMU orientation
        pose.pose.position.x = 0.0
        pose.pose.position.y = 0.0
        pose.pose.position.z = 0.0

        # Set covariance based on sensor uncertainties
        pose.covariance = [0.1, 0, 0, 0, 0, 0,  # Position covariance
                          0, 0.1, 0, 0, 0, 0,
                          0, 0, 0.1, 0, 0, 0,
                          0, 0, 0, 0.1, 0, 0,  # Orientation covariance
                          0, 0, 0, 0, 0.1, 0,
                          0, 0, 0, 0, 0, 0.1]

        return pose

    def create_fused_pointcloud(self):
        """Create a fused point cloud from camera and LIDAR data"""
        # In a real implementation, this would project camera pixels
        # into 3D space using LIDAR depth information
        pass
```

## Physics-Based Sensor Simulation

### Realistic Sensor Noise Modeling

```python
import numpy as np
from scipy.spatial.transform import Rotation as R

class PhysicsBasedSensorSimulator:
    def __init__(self):
        # Sensor characteristics
        self.camera_noise_params = {
            'gaussian_noise': 0.02,
            'poisson_noise': 0.01,
            'uniform_distortion': 0.05
        }

        self.lidar_noise_params = {
            'range_bias': 0.02,  # 2cm bias
            'range_noise': 0.01,  # 1cm noise
            'angular_bias': 0.001,  # 0.057 degrees
            'angular_noise': 0.0005  # 0.029 degrees
        }

        self.imu_noise_params = {
            'accel_bias': 0.01,  # 1% of gravity
            'accel_noise': 0.001,
            'gyro_bias': 0.001,  # rad/s
            'gyro_noise': 0.0001,
            'mag_bias': 0.1,    # micro Tesla
            'mag_noise': 0.01
        }

    def add_camera_noise(self, image):
        """Add realistic noise to camera images"""
        # Add Gaussian noise
        gaussian_noise = np.random.normal(
            0, self.camera_noise_params['gaussian_noise'], image.shape)
        noisy_image = image.astype(np.float32) + gaussian_noise * 255

        # Add Poisson noise (signal-dependent)
        poisson_noise = np.random.poisson(noisy_image / 255.0) * 255 - noisy_image
        noisy_image += poisson_noise * self.camera_noise_params['poisson_noise']

        # Ensure values are in valid range
        noisy_image = np.clip(noisy_image, 0, 255).astype(np.uint8)

        return noisy_image

    def add_lidar_noise(self, ranges, angles):
        """Add realistic noise to LIDAR measurements"""
        noisy_ranges = []

        for i, (range_val, angle) in enumerate(zip(ranges, angles)):
            if range_val >= 10.0:  # Maximum range
                noisy_ranges.append(range_val)
                continue

            # Add bias and noise
            biased_range = range_val + self.lidar_noise_params['range_bias']
            noisy_range = biased_range + np.random.normal(0, self.lidar_noise_params['range_noise'])

            # Range-dependent noise (closer objects have less noise)
            range_dependent_noise = self.lidar_noise_params['range_noise'] * (1.0 - range_val / 10.0)
            final_range = noisy_range + np.random.normal(0, range_dependent_noise)

            noisy_ranges.append(max(0.1, final_range))  # Minimum range constraint

        return noisy_ranges

    def add_imu_noise(self, accel_true, gyro_true, dt):
        """Add realistic noise to IMU measurements"""
        # Accelerometer noise
        accel_bias = np.random.normal(0, self.imu_noise_params['accel_bias'], 3)
        accel_noise = np.random.normal(0, self.imu_noise_params['accel_noise'], 3)
        accel_measurement = accel_true + accel_bias + accel_noise

        # Gyroscope noise
        gyro_bias = np.random.normal(0, self.imu_noise_params['gyro_bias'], 3)
        gyro_noise = np.random.normal(0, self.imu_noise_params['gyro_noise'], 3)
        gyro_measurement = gyro_true + gyro_bias + gyro_noise

        return accel_measurement, gyro_measurement
```

## Environmental Effects on Sensors

### Weather Simulation

```python
class WeatherEffectSimulator:
    def __init__(self):
        self.weather_conditions = {
            'clear': {'visibility': 100.0, 'precipitation': 0.0},
            'rain_light': {'visibility': 30.0, 'precipitation': 2.0},
            'rain_heavy': {'visibility': 10.0, 'precipitation': 8.0},
            'fog_light': {'visibility': 20.0, 'precipitation': 0.0},
            'fog_dense': {'visibility': 5.0, 'precipitation': 0.0}
        }

        self.current_weather = 'clear'

    def set_weather(self, condition):
        """Set the current weather condition"""
        if condition in self.weather_conditions:
            self.current_weather = condition
            return True
        return False

    def affect_camera(self, image):
        """Apply weather effects to camera images"""
        condition = self.weather_conditions[self.current_weather]

        if 'rain' in self.current_weather:
            # Rain effect: reduce contrast and add water droplets
            image = self.apply_rain_effect(image, condition['precipitation'])
        elif 'fog' in self.current_weather:
            # Fog effect: reduce visibility and contrast
            image = self.apply_fog_effect(image, condition['visibility'])

        return image

    def affect_lidar(self, ranges, angles):
        """Apply weather effects to LIDAR measurements"""
        condition = self.weather_conditions[self.current_weather]

        affected_ranges = []
        for r in ranges:
            if r >= 10.0:  # Maximum range
                affected_ranges.append(r)
                continue

            # In bad weather, effective range decreases
            visibility_factor = min(condition['visibility'] / 10.0, 1.0)
            effective_range = r * visibility_factor

            # Add weather-related noise
            if 'rain' in self.current_weather:
                weather_noise = np.random.normal(0, 0.05)  # More noise in rain
            elif 'fog' in self.current_weather:
                weather_noise = np.random.normal(0, 0.03)  # Some noise in fog
            else:
                weather_noise = 0

            affected_ranges.append(max(0.1, effective_range + weather_noise))

        return affected_ranges

    def apply_rain_effect(self, image, precipitation_level):
        """Apply rain effect to image"""
        # Reduce visibility
        alpha = 1.0 - (precipitation_level / 10.0) * 0.3
        image = (image * alpha).astype(np.uint8)

        # Add rain streaks (simplified)
        rain_intensity = precipitation_level / 10.0
        rain_pixels = np.random.random(image.shape[:2]) < rain_intensity * 0.05
        image[rain_pixels] = [200, 200, 255]  # Light blue streaks

        return image

    def apply_fog_effect(self, image, visibility):
        """Apply fog effect to image"""
        # Calculate fog factor based on visibility
        fog_factor = 1.0 - (visibility / 100.0)

        # Apply fog by blending with gray
        fog_color = np.array([128, 128, 128], dtype=np.uint8)
        fogged_image = (image * (1 - fog_factor) + fog_color * fog_factor).astype(np.uint8)

        return fogged_image
```

## Sensor Calibration Simulation

### Calibration Process Simulation

```python
class SensorCalibrationSimulator:
    def __init__(self):
        # True calibration parameters
        self.true_intrinsics = np.array([
            [500.0, 0.0, 320.0],  # fx, 0, cx
            [0.0, 500.0, 240.0],  # 0, fy, cy
            [0.0, 0.0, 1.0]       # 0, 0, 1
        ])

        self.true_distortion = np.array([0.1, -0.2, 0.001, 0.002, 0.0])  # k1, k2, p1, p2, k3

        # Initial calibration guess (with errors)
        self.calibrated_intrinsics = np.array([
            [495.0, 0.0, 322.0],
            [0.0, 498.0, 238.0],
            [0.0, 0.0, 1.0]
        ])

        self.calibrated_distortion = np.array([0.09, -0.18, 0.002, 0.001, 0.001])

    def simulate_calibration_target(self, num_points=50):
        """Simulate a calibration target (checkerboard) in 3D"""
        # Create a checkerboard pattern in 3D
        rows, cols = 8, 6
        square_size = 0.025  # 2.5 cm squares

        # Generate 3D points of the checkerboard
        obj_points = []
        for j in range(rows):
            for i in range(cols):
                obj_points.append([i * square_size, j * square_size, 0])

        obj_points = np.array(obj_points, dtype=np.float32)

        # Generate multiple views by applying random transformations
        views = []
        for _ in range(10):  # 10 different views
            # Random rotation and translation
            rvec = np.random.uniform(-0.5, 0.5, 3)  # Small rotations
            tvec = np.random.uniform(-0.1, 0.1, 3)  # Small translations

            # Project 3D points to 2D using true intrinsics
            projected_points, _ = cv2.projectPoints(
                obj_points, rvec, tvec,
                self.true_intrinsics, self.true_distortion
            )

            views.append({
                'obj_points': obj_points.copy(),
                'img_points': projected_points.reshape(-1, 2),
                'rvec': rvec,
                'tvec': tvec
            })

        return views

    def calibrate_camera(self, views):
        """Simulate camera calibration process"""
        # Collect all object points and image points
        all_obj_points = []
        all_img_points = []

        for view in views:
            all_obj_points.append(view['obj_points'])
            all_img_points.append(view['img_points'])

        # Perform calibration (would normally use cv2.calibrateCamera)
        # Here we'll just show the process
        ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(
            all_obj_points, all_img_points,
            (640, 480), None, None
        )

        return ret, mtx, dist, rvecs, tvecs

    def evaluate_calibration(self, calibrated_intrinsics, calibrated_distortion):
        """Evaluate the quality of calibration"""
        # Calculate reprojection error
        test_points_3d = np.array([[0, 0, 0], [0.1, 0, 0], [0, 0.1, 0], [0.1, 0.1, 0]], dtype=np.float32)

        # Project using true parameters
        true_2d, _ = cv2.projectPoints(
            test_points_3d, np.zeros(3), np.zeros(3),
            self.true_intrinsics, self.true_distortion
        )

        # Project using calibrated parameters
        calibrated_2d, _ = cv2.projectPoints(
            test_points_3d, np.zeros(3), np.zeros(3),
            calibrated_intrinsics, calibrated_distortion
        )

        # Calculate reprojection error
        error = np.mean(np.linalg.norm(true_2d - calibrated_2d, axis=2))

        return error
```

## Learning Objectives

After completing this chapter, you will be able to:
- Simulate various types of sensors used in robotics
- Implement realistic noise models for sensor data
- Perform sensor fusion to combine data from multiple sensors
- Account for environmental effects on sensor performance
- Understand the calibration process for sensors

## Hands-on Exercise

Create a simulation node that publishes data from multiple sensors (camera, LIDAR, IMU) with realistic noise models. Implement a simple sensor fusion algorithm that combines these sensor readings to estimate the robot's state.

:::tip
Use `rqt_plot` or `plotjuggler` to visualize sensor data streams and verify that your noise models produce realistic-looking signals.
:::

:::note
Consider the computational cost of sensor simulation, especially when simulating multiple sensors at high frequencies.
:::