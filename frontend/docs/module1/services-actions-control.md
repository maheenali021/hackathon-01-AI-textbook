---
sidebar_position: 5
title: "Chapter 5: Services and Actions for Robot Control"
---

# Services and Actions for Robot Control

## Understanding Request-Response Communication

Services and actions provide request-response communication patterns in ROS 2, essential for robot control and task execution.

## Services

### Service Definition

Services provide synchronous request-response communication. Define services using `.srv` files:

```txt
# AddTwoInts.srv
int64 a
int64 b
---
int64 sum
```

### Service Server Implementation

```python
import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts

class MinimalService(Node):
    def __init__(self):
        super().__init__('minimal_service')
        self.srv = self.create_service(AddTwoInts, 'add_two_ints', self.add_two_ints_callback)

    def add_two_ints_callback(self, request, response):
        response.sum = request.a + request.b
        self.get_logger().info(f'Returning: {response.sum}')
        return response

def main(args=None):
    rclpy.init(args=args)
    minimal_service = MinimalService()
    rclpy.spin(minimal_service)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Service Client Implementation

```python
import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts

class MinimalClient(Node):
    def __init__(self):
        super().__init__('minimal_client')
        self.cli = self.create_client(AddTwoInts, 'add_two_ints')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting again...')
        self.req = AddTwoInts.Request()

    def send_request(self, a, b):
        self.req.a = a
        self.req.b = b
        self.future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()

def main(args=None):
    rclpy.init(args=args)
    minimal_client = MinimalClient()
    response = minimal_client.send_request(1, 2)
    minimal_client.get_logger().info(f'Result: {response.sum}')
    minimal_client.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Actions

### Action Definition

Actions handle long-running tasks with feedback and goals. Define actions using `.action` files:

```txt
# Fibonacci.action
int32 order
---
int32[] sequence
---
int32[] partial_sequence
```

### Action Server Implementation

```python
import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node
from example_interfaces.action import Fibonacci

class FibonacciActionServer(Node):
    def __init__(self):
        super().__init__('fibonacci_action_server')
        self._action_server = ActionServer(
            self,
            Fibonacci,
            'fibonacci',
            self.execute_callback)

    def execute_callback(self, goal_handle):
        self.get_logger().info('Executing goal...')

        feedback_msg = Fibonacci.Feedback()
        feedback_msg.partial_sequence = [0, 1]

        for i in range(1, goal_handle.request.order):
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                self.get_logger().info('Goal canceled')
                return Fibonacci.Result()

            feedback_msg.partial_sequence.append(
                feedback_msg.partial_sequence[i] + feedback_msg.partial_sequence[i-1])

            goal_handle.publish_feedback(feedback_msg)
            time.sleep(1)

        goal_handle.succeed()
        result = Fibonacci.Result()
        result.sequence = feedback_msg.partial_sequence
        return result

def main(args=None):
    rclpy.init(args=args)
    fibonacci_action_server = FibonacciActionServer()
    rclpy.spin(fibonacci_action_server)
    fibonacci_action_server.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Action Client Implementation

```python
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from example_interfaces.action import Fibonacci

class FibonacciActionClient(Node):
    def __init__(self):
        super().__init__('fibonacci_action_client')
        self._action_client = ActionClient(
            self,
            Fibonacci,
            'fibonacci')

    def send_goal(self, order):
        goal_msg = Fibonacci.Goal()
        goal_msg.order = order

        self._action_client.wait_for_server()
        self._send_goal_future = self._action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback)

        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected')
            return

        self.get_logger().info('Goal accepted')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info(f'Received feedback: {feedback.partial_sequence}')

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info(f'Result: {result.sequence}')

def main(args=None):
    rclpy.init(args=args)
    action_client = FibonacciActionClient()
    action_client.send_goal(10)
    rclpy.spin(action_client)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Robot Control Applications

### Movement Control with Services

```python
# Movement control service
from geometry_msgs.msg import Twist

class MovementControlService(Node):
    def __init__(self):
        super().__init__('movement_control_service')
        self.cmd_vel_publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.srv = self.create_service(
            MovementCommand,
            'move_robot',
            self.move_callback)

    def move_callback(self, request, response):
        cmd = Twist()
        cmd.linear.x = request.linear_speed
        cmd.angular.z = request.angular_speed

        self.cmd_vel_publisher.publish(cmd)
        response.success = True
        response.message = f'Moved with linear: {request.linear_speed}, angular: {request.angular_speed}'
        return response
```

### Navigation with Actions

Actions are ideal for navigation tasks that take time to complete:

:::note
Use actions for navigation goals rather than services because navigation can take significant time and users may want periodic feedback on progress.
:::

## Best Practices

### Service Design

- Keep services for short, synchronous operations
- Use appropriate error handling
- Consider timeout values for clients

### Action Design

- Use actions for long-running operations
- Provide meaningful feedback messages
- Handle cancellation requests properly

### Performance Considerations

- Services block until completion
- Actions provide non-blocking communication
- Consider the impact of synchronous operations on system performance

## Learning Objectives

After completing this chapter, you will be able to:
- Implement service servers and clients
- Create and use action servers and clients
- Choose appropriate communication patterns for robot control
- Handle errors and timeouts in service and action communication

## Hands-on Exercise

Create a robot control system using both services for immediate commands and actions for navigation tasks. Implement a client that uses both communication patterns.

:::tip
Use `ros2 service list` and `ros2 action list` to discover available services and actions on your system.
:::