---
sidebar_position: 4
title: "Chapter 4: Publishers and Subscribers in ROS 2"
---

# Publishers and Subscribers in ROS 2

## Understanding Publish-Subscribe Pattern

The publish-subscribe pattern is the fundamental communication mechanism in ROS 2. Publishers send messages to topics, and subscribers receive messages from topics.

## Creating Publishers

### Basic Publisher Structure

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class MinimalPublisher(Node):
    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(String, 'topic', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = String()
        msg.data = f'Hello World: {self.i}'
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: "{msg.data}"')
        self.i += 1

def main(args=None):
    rclpy.init(args=args)
    minimal_publisher = MinimalPublisher()
    rclpy.spin(minimal_publisher)
    minimal_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Advanced Publisher Features

```python
from rclpy.qos import QoSProfile, HistoryPolicy, ReliabilityPolicy

# Create a publisher with custom QoS settings
qos_profile = QoSProfile(
    depth=10,
    history=HistoryPolicy.KEEP_LAST,
    reliability=ReliabilityPolicy.RELIABLE
)

publisher = self.create_publisher(String, 'topic', qos_profile)
```

## Creating Subscribers

### Basic Subscriber Structure

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class MinimalSubscriber(Node):
    def __init__(self):
        super().__init__('minimal_subscriber')
        self.subscription = self.create_subscription(
            String,
            'topic',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        self.get_logger().info(f'I heard: "{msg.data}"')

def main(args=None):
    rclpy.init(args=args)
    minimal_subscriber = MinimalSubscriber()
    rclpy.spin(minimal_subscriber)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Message Types and Serialization

### Built-in Message Types

ROS 2 provides various built-in message types:

- **std_msgs**: Basic data types (String, Int32, Float64, etc.)
- **geometry_msgs**: Geometric primitives (Point, Pose, Twist, etc.)
- **sensor_msgs**: Sensor data (LaserScan, Image, JointState, etc.)

### Custom Message Types

Create custom messages by defining `.msg` files in the `msg/` directory:

```txt
# CustomMessage.msg
string name
int32 id
float64[] values
geometry_msgs/Pose pose
```

## Advanced Patterns

### Multiple Publishers and Subscribers

```python
class MultiTopicNode(Node):
    def __init__(self):
        super().__init__('multi_topic_node')

        # Multiple publishers
        self.pub1 = self.create_publisher(String, 'topic1', 10)
        self.pub2 = self.create_publisher(Int32, 'topic2', 10)

        # Multiple subscribers
        self.sub1 = self.create_subscription(String, 'topic1', self.callback1, 10)
        self.sub2 = self.create_subscription(Int32, 'topic2', self.callback2, 10)
```

### Conditional Publishing

```python
def conditional_publish(self, data):
    if self.should_publish:
        self.publisher.publish(data)
```

## Best Practices

### Error Handling

```python
try:
    self.publisher.publish(msg)
except Exception as e:
    self.get_logger().error(f'Failed to publish message: {e}')
```

### Memory Management

:::tip
Use `rclpy.qos.QoSHistoryPolicy.KEEP_LAST` with appropriate depth to manage memory usage when publishing high-frequency messages.
:::

### Thread Safety

Subscribers run in the same thread as the executor, so avoid long-running operations in callbacks.

## Quality of Service Considerations

Different QoS settings affect performance and reliability:

- **Reliable vs Best Effort**: Choose based on whether message loss is acceptable
- **Keep Last vs Keep All**: Consider memory usage for high-frequency topics
- **Deadline and Lifespan**: Configure for time-sensitive applications

## Learning Objectives

After completing this chapter, you will be able to:
- Create publishers and subscribers in ROS 2
- Configure Quality of Service settings appropriately
- Handle different message types
- Implement advanced publish-subscribe patterns

## Hands-on Exercise

Create a publisher that sends sensor data and a subscriber that processes and visualizes the data. Experiment with different QoS settings to observe their effects.

:::warning
Remember to always call `rclpy.shutdown()` when your node exits to properly clean up resources.
:::