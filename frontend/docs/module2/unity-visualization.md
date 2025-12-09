---
sidebar_position: 3
title: "Chapter 3: Unity Visualization for Robotics"
---

# Unity Visualization for Robotics

## Introduction to Unity for Robotics

Unity is a powerful game engine that can be used for robotics simulation and visualization. It provides high-quality rendering capabilities and physics simulation, making it suitable for creating realistic robot environments.

## Unity Robotics Setup

### Installing Unity Robotics Hub

Unity provides the Robotics Hub to streamline the connection between Unity and ROS/ROS 2:

1. Download Unity Hub from unity.com
2. Install Unity Editor (2021.3 LTS or newer recommended)
3. Install the Unity Robotics Hub package

### Unity ROS TCP Connector

The Unity ROS TCP Connector enables communication between Unity and ROS:

```csharp
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Std;
using UnityEngine;

public class UnityRobotController : MonoBehaviour
{
    ROSConnection ros;
    public string topicName = "unity_robot_command";

    void Start()
    {
        // Get the ROS connection system
        ros = ROSConnection.GetOrCreateInstance();
        ros.RegisterPublisher<UInt8Msg>(topicName);
    }

    void Update()
    {
        // Example: Send a command to ROS
        if (Input.GetKeyDown(KeyCode.Space))
        {
            UInt8Msg command = new UInt8Msg();
            command.data = 1; // Send command value
            ros.Publish(topicName, command);
        }
    }
}
```

## Creating Robot Environments

### Environment Setup

```csharp
using UnityEngine;

public class RobotEnvironment : MonoBehaviour
{
    public GameObject robotPrefab;
    public Transform spawnPoint;
    public Material[] materials;

    void Start()
    {
        // Spawn robot in environment
        Instantiate(robotPrefab, spawnPoint.position, spawnPoint.rotation);

        // Set up environment materials
        SetupEnvironment();
    }

    void SetupEnvironment()
    {
        // Create obstacles, platforms, etc.
        CreateObstacles();
        SetLighting();
    }

    void CreateObstacles()
    {
        // Create various obstacles for robot navigation
        for (int i = 0; i < 10; i++)
        {
            Vector3 pos = new Vector3(
                Random.Range(-10f, 10f),
                0.5f,
                Random.Range(-10f, 10f)
            );

            GameObject obstacle = GameObject.CreatePrimitive(PrimitiveType.Cube);
            obstacle.transform.position = pos;
            obstacle.transform.localScale = new Vector3(
                Random.Range(0.5f, 2f),
                Random.Range(1f, 3f),
                Random.Range(0.5f, 2f)
            );

            // Add material
            Renderer renderer = obstacle.GetComponent<Renderer>();
            renderer.material = materials[Random.Range(0, materials.Length)];
        }
    }

    void SetLighting()
    {
        // Configure lighting for the scene
        RenderSettings.ambientLight = Color.gray;
        RenderSettings.fog = true;
        RenderSettings.fogColor = Color.lightGray;
        RenderSettings.fogDensity = 0.01f;
    }
}
```

## Physics Simulation

### Unity Physics vs Real Robot Dynamics

Unity's physics engine can approximate real robot dynamics:

```csharp
using UnityEngine;

[RequireComponent(typeof(Rigidbody))]
public class RobotDynamics : MonoBehaviour
{
    public float maxForce = 10f;
    public float maxTorque = 5f;
    public float frictionCoefficient = 0.1f;

    private Rigidbody rb;

    void Start()
    {
        rb = GetComponent<Rigidbody>();
        ConfigurePhysics();
    }

    void ConfigurePhysics()
    {
        rb.mass = 10f; // Adjust based on real robot mass
        rb.drag = 0.1f; // Linear drag
        rb.angularDrag = 0.05f; // Angular drag
        rb.interpolation = RigidbodyInterpolation.Interpolate;
    }

    public void ApplyMovement(Vector3 linearVelocity, Vector3 angularVelocity)
    {
        // Apply forces to simulate robot movement
        rb.velocity = linearVelocity;
        rb.angularVelocity = angularVelocity;
    }

    public void ApplyControlInputs(float forwardForce, float turnTorque)
    {
        // Apply forces based on control inputs
        Vector3 forwardForceVector = transform.forward * forwardForce;
        rb.AddForce(forwardForceVector, ForceMode.Force);

        rb.AddTorque(transform.up * turnTorque, ForceMode.Torque);
    }
}
```

## Sensor Simulation

### Camera Simulation

```csharp
using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Sensor;

public class CameraSensor : MonoBehaviour
{
    public Camera camera;
    public string imageTopic = "camera/image_raw";
    public int width = 640;
    public int height = 480;

    private ROSConnection ros;
    private RenderTexture renderTexture;

    void Start()
    {
        ros = ROSConnection.GetOrCreateInstance();

        // Create render texture for camera
        renderTexture = new RenderTexture(width, height, 24);
        camera.targetTexture = renderTexture;
    }

    void Update()
    {
        if (Time.frameCount % 30 == 0) // Send image every 30 frames
        {
            SendImage();
        }
    }

    void SendImage()
    {
        // Capture image from camera
        Texture2D tex = new Texture2D(width, height, TextureFormat.RGB24, false);
        RenderTexture.active = renderTexture;
        tex.ReadPixels(new Rect(0, 0, width, height), 0, 0);
        tex.Apply();

        // Convert to byte array and publish
        byte[] imageData = tex.EncodeToPNG();

        // Create ROS message (simplified)
        // In practice, you would create a proper sensor_msgs/Image message

        Destroy(tex);
    }
}
```

### LIDAR Simulation

```csharp
using UnityEngine;
using System.Collections.Generic;

public class LIDARSensor : MonoBehaviour
{
    public int numRays = 360;
    public float maxRange = 10f;
    public float minRange = 0.1f;
    public string lidarTopic = "scan";

    private ROSConnection ros;

    void Start()
    {
        ros = ROSConnection.GetOrCreateInstance();
    }

    void Update()
    {
        if (Time.frameCount % 10 == 0) // Sample every 10 frames
        {
            ScanEnvironment();
        }
    }

    void ScanEnvironment()
    {
        List<float> ranges = new List<float>();

        for (int i = 0; i < numRays; i++)
        {
            float angle = (i * 360f / numRays) * Mathf.Deg2Rad;
            Vector3 direction = new Vector3(
                Mathf.Cos(angle),
                0,
                Mathf.Sin(angle)
            );

            RaycastHit hit;
            if (Physics.Raycast(transform.position, direction, out hit, maxRange))
            {
                ranges.Add(hit.distance);
            }
            else
            {
                ranges.Add(maxRange); // No obstacle detected
            }
        }

        // Publish scan data to ROS (implementation depends on ROS message format)
        PublishScanData(ranges);
    }

    void PublishScanData(List<float> ranges)
    {
        // Publish to ROS topic
        // Implementation would create and publish sensor_msgs/LaserScan message
    }
}
```

## Human-Robot Interaction

### AR/VR Integration

Unity excels at creating immersive AR/VR experiences for human-robot interaction:

```csharp
using UnityEngine;
using UnityEngine.XR;

public class HumanRobotInterface : MonoBehaviour
{
    public GameObject robotAvatar;
    public Transform userHead;
    public float interactionDistance = 3f;

    void Update()
    {
        // Handle VR/AR interactions
        HandleInteractions();
        UpdateRobotBehavior();
    }

    void HandleInteractions()
    {
        // Raycast from user viewpoint
        Vector3 rayOrigin = userHead.position;
        Vector3 rayDirection = userHead.forward;

        RaycastHit hit;
        if (Physics.Raycast(rayOrigin, rayDirection, out hit, interactionDistance))
        {
            if (hit.collider.CompareTag("Robot"))
            {
                // User is looking at robot
                HighlightRobot(true);

                if (Input.GetButtonDown("Fire1")) // Trigger button
                {
                    SendCommandToRobot();
                }
            }
            else
            {
                HighlightRobot(false);
            }
        }
        else
        {
            HighlightRobot(false);
        }
    }

    void HighlightRobot(bool highlight)
    {
        // Visual feedback for robot selection
        Renderer robotRenderer = robotAvatar.GetComponent<Renderer>();
        if (highlight)
        {
            robotRenderer.material.color = Color.yellow;
        }
        else
        {
            robotRenderer.material.color = Color.white;
        }
    }

    void SendCommandToRobot()
    {
        // Send command via ROS
        // Implementation would publish to robot control topic
    }

    void UpdateRobotBehavior()
    {
        // Update robot behavior based on simulation
    }
}
```

## Performance Optimization

### Level of Detail (LOD) System

```csharp
using UnityEngine;

public class RobotLODSystem : MonoBehaviour
{
    public GameObject[] lodLevels;
    public float[] lodDistances;
    public Transform viewer;

    void Start()
    {
        viewer = Camera.main.transform;
    }

    void Update()
    {
        UpdateLOD();
    }

    void UpdateLOD()
    {
        float distance = Vector3.Distance(transform.position, viewer.position);

        for (int i = 0; i < lodLevels.Length; i++)
        {
            if (distance < lodDistances[i])
            {
                ActivateLOD(i);
                return;
            }
        }

        // If beyond max distance, hide robot
        foreach (GameObject lod in lodLevels)
        {
            lod.SetActive(false);
        }
    }

    void ActivateLOD(int level)
    {
        for (int i = 0; i < lodLevels.Length; i++)
        {
            lodLevels[i].SetActive(i == level);
        }
    }
}
```

## Integration with ROS/ROS 2

### Message Handling

```csharp
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Geometry;
using RosMessageTypes.Std;

public class ROSMessageHandler : MonoBehaviour
{
    ROSConnection ros;

    // Topics
    string cmdVelTopic = "cmd_vel";
    string jointStatesTopic = "joint_states";

    void Start()
    {
        ros = ROSConnection.GetOrCreateInstance();

        // Subscribe to ROS topics
        ros.Subscribe<TwistMsg>(cmdVelTopic, CmdVelCallback);
        ros.Subscribe<Float32MultiArrayMsg>(jointStatesTopic, JointStatesCallback);
    }

    void CmdVelCallback(TwistMsg twist)
    {
        // Handle velocity commands
        Vector3 linear = new Vector3(twist.linear.x, twist.linear.y, twist.linear.z);
        Vector3 angular = new Vector3(twist.angular.x, twist.angular.y, twist.angular.z);

        // Apply to robot
        ApplyVelocityCommand(linear, angular);
    }

    void JointStatesCallback(Float32MultiArrayMsg jointStates)
    {
        // Update joint positions in Unity
        UpdateJointPositions(jointStates.data);
    }

    void ApplyVelocityCommand(Vector3 linear, Vector3 angular)
    {
        // Apply velocity to Unity robot
        // Implementation depends on robot dynamics
    }

    void UpdateJointPositions(float[] positions)
    {
        // Update each joint in the Unity robot model
        // Implementation would iterate through robot joints
    }
}
```

## Best Practices

### Scene Organization

- Use layers and tags for efficient physics and rendering
- Implement object pooling for frequently instantiated objects
- Use occlusion culling for large environments
- Optimize draw calls with batching

### Asset Management

- Use appropriate polygon counts for real-time performance
- Implement texture atlasing for improved rendering
- Use LOD systems for distant objects
- Optimize animations with compression

## Learning Objectives

After completing this chapter, you will be able to:
- Set up Unity for robotics simulation and visualization
- Implement sensor simulation in Unity
- Create realistic robot environments
- Integrate Unity with ROS/ROS 2 for bidirectional communication
- Optimize Unity scenes for real-time robotics applications

## Hands-on Exercise

Create a simple Unity scene with a robot model that can receive movement commands from ROS and visualize its environment. Implement basic camera and LIDAR sensors that publish data back to ROS.

:::tip
Use Unity's built-in Profiler to identify performance bottlenecks in your robotics simulation.
:::

:::note
Consider using Unity's Recorder package to capture simulation data for later analysis and debugging.
:::