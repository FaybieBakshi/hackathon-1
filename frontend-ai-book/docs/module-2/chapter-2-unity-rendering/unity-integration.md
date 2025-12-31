---
sidebar_position: 7
title: "Unity-Gazebo Integration"
---

# Unity-Gazebo Integration Techniques

## Overview

This section covers the essential techniques for integrating Unity with Gazebo to create synchronized digital twin environments. You'll learn how to establish communication between the two systems and maintain real-time synchronization between physics simulation and visual rendering.

## Integration Architecture

### Communication Patterns

The Unity-Gazebo integration typically follows this architecture:

```
Gazebo Physics Engine ←→ ROS Bridge ←→ Unity ROS TCP Endpoint ←→ Unity Application
```

### Key Components

1. **Gazebo Simulation**: Handles physics, collisions, and dynamics
2. **ROS Bridge**: Translates between Gazebo and ROS messages
3. **Unity ROS TCP Endpoint**: Receives and sends ROS messages to Unity
4. **Unity Application**: Renders the visual representation

## Setting Up the ROS Bridge

### Installing ROS Bridge

```bash
# Install ROS bridge packages
sudo apt install ros-humble-rosbridge-suite
sudo apt install ros-humble-gazebo-ros-pkgs
```

### Launching the Bridge

```bash
# Start ROS bridge server
ros2 launch rosbridge_server rosbridge_websocket_launch.xml

# Or with specific port
ros2 run rosbridge_server rosbridge_websocket -- --port 9090
```

## Unity ROS TCP Endpoint

### Installation and Setup

1. **Download Unity Robotics Package**:
   - Use Unity Package Manager to install `ROS TCP Connector`
   - Or download from Unity Asset Store

2. **Configure the Endpoint**:
   ```csharp
   using Unity.Robotics.ROSTCPConnector;

   public class RobotUnitySetup : MonoBehaviour
   {
       [Header("ROS Connection")]
       public string rosIPAddress = "127.0.0.1";
       public int rosPort = 9090;

       void Start()
       {
           // Initialize ROS connection
           ROSConnection.GetOrCreateInstance().Initialize(rosIPAddress, rosPort);
       }
   }
   ```

### Message Types and Communication

Unity supports various ROS message types:

```csharp
using Unity.Robotics.ROSTCPConnector;
using Unity.Robotics.ROSTCPConnector.MessageTypes.Std;
using Unity.Robotics.ROSTCPConnector.MessageTypes.Sensor;
using Unity.Robotics.ROSTCPConnector.MessageTypes.Geometry;

public class RobotMessageHandler : MonoBehaviour
{
    [Header("Topic Names")]
    public string jointStatesTopic = "/joint_states";
    public string modelStatesTopic = "/gazebo/model_states";
    public string cmdVelTopic = "/cmd_vel";

    void Start()
    {
        var ros = ROSConnection.GetOrCreateInstance();

        // Subscribe to Gazebo model states
        ros.Subscribe<ModelStatesMsg>(modelStatesTopic, OnModelStatesReceived);

        // Subscribe to joint states
        ros.Subscribe<JointStateMsg>(jointStatesTopic, OnJointStatesReceived);
    }

    void OnModelStatesReceived(ModelStatesMsg msg)
    {
        // Update Unity objects based on Gazebo model states
        for (int i = 0; i < msg.name.Count; i++)
        {
            string modelName = msg.name[i];
            var position = msg.pose[i].position;
            var orientation = msg.pose[i].orientation;

            // Find corresponding Unity object and update its transform
            GameObject model = GameObject.Find(modelName);
            if (model != null)
            {
                model.transform.position = new Vector3(
                    (float)position.x,
                    (float)position.z,  // Unity Y-up
                    (float)position.y   // Swap Y and Z
                );

                model.transform.rotation = new Quaternion(
                    (float)orientation.x,
                    (float)orientation.z,
                    (float)orientation.y,
                    (float)orientation.w
                );
            }
        }
    }

    void OnJointStatesReceived(JointStateMsg msg)
    {
        // Update joint positions in Unity
        for (int i = 0; i < msg.name.Count; i++)
        {
            string jointName = msg.name[i];
            double position = msg.position[i];

            // Find joint in Unity hierarchy and update
            Transform joint = transform.FindRecursive(jointName);
            if (joint != null)
            {
                // Apply joint position based on joint type
                ApplyJointPosition(joint, jointName, position);
            }
        }
    }

    void ApplyJointPosition(Transform jointTransform, string jointName, double position)
    {
        // Implementation depends on joint type
        // For revolute joints: rotate around axis
        // For prismatic joints: translate along axis
    }
}
```

## Synchronization Techniques

### 1. Transform Synchronization

Keep Unity transforms synchronized with Gazebo physics:

```csharp
using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using Unity.Robotics.ROSTCPConnector.MessageTypes.Gazebo;

public class TransformSynchronizer : MonoBehaviour
{
    [Header("Synchronization Settings")]
    public float syncRate = 60.0f; // Hz
    public bool enableInterpolation = true;

    private float syncInterval;
    private float lastSyncTime;

    void Start()
    {
        syncInterval = 1.0f / syncRate;
        lastSyncTime = Time.time;

        // Subscribe to Gazebo model states
        var ros = ROSConnection.GetOrCreateInstance();
        ros.Subscribe<ModelStatesMsg>("/gazebo/model_states", OnModelStatesReceived);
    }

    void Update()
    {
        if (Time.time - lastSyncTime >= syncInterval)
        {
            SyncTransforms();
            lastSyncTime = Time.time;
        }
    }

    void SyncTransforms()
    {
        // Smooth interpolation between received positions
        if (enableInterpolation)
        {
            InterpolateTransforms();
        }
    }

    void OnModelStatesReceived(ModelStatesMsg msg)
    {
        // Store received transforms for interpolation
        StoreReceivedTransforms(msg);
    }

    void StoreReceivedTransforms(ModelStatesMsg msg)
    {
        // Cache the received transforms
        // Implement interpolation logic here
    }

    void InterpolateTransforms()
    {
        // Smooth transition between physics states
        // This prevents jerky movements due to network latency
    }
}
```

### 2. Time Synchronization

Ensure Unity and Gazebo maintain consistent time:

```csharp
using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using Unity.Robotics.ROSTCPConnector.MessageTypes.Std;

public class TimeSynchronizer : MonoBehaviour
{
    [Header("Time Sync Settings")]
    public bool enableTimeSync = true;
    public float maxTimeDifference = 0.1f; // seconds

    private double gazeboTime = 0.0;
    private double lastGazeboTime = 0.0;
    private double unityTime = 0.0;

    void Start()
    {
        var ros = ROSConnection.GetOrCreateInstance();
        ros.Subscribe<ClockMsg>("/clock", OnClockReceived);
    }

    void OnClockReceived(ClockMsg clockMsg)
    {
        gazeboTime = clockMsg.clock.sec + clockMsg.clock.nsec / 1000000000.0;

        if (enableTimeSync)
        {
            double timeDiff = gazeboTime - lastGazeboTime;
            if (timeDiff > maxTimeDifference)
            {
                // Handle large time jumps
                Debug.LogWarning($"Large time jump detected: {timeDiff} seconds");
            }

            lastGazeboTime = gazeboTime;
        }
    }

    void Update()
    {
        unityTime += Time.deltaTime;
    }

    public double GetSynchronizedTime()
    {
        if (enableTimeSync)
        {
            return gazeboTime;
        }
        else
        {
            return unityTime;
        }
    }
}
```

## Robot Model Integration

### URDF to Unity Conversion

Convert ROS URDF models to Unity-compatible formats:

```csharp
using UnityEngine;
using System.Xml;
using System.Collections.Generic;

public class URDFToUnityConverter : MonoBehaviour
{
    [Header("URDF Settings")]
    public TextAsset urdfFile;
    public GameObject robotPrefab;

    [System.Serializable]
    public class LinkData
    {
        public string name;
        public Vector3 position;
        public Quaternion rotation;
        public Vector3 size;
        public string geometryType;
        public string meshFile;
    }

    [System.Serializable]
    public class JointData
    {
        public string name;
        public string type;
        public string parentLink;
        public string childLink;
        public Vector3 axis;
        public Vector3 originPosition;
        public Quaternion originRotation;
    }

    private List<LinkData> links = new List<LinkData>();
    private List<JointData> joints = new List<JointData>();

    void Start()
    {
        if (urdfFile != null)
        {
            ParseURDF(urdfFile.text);
            CreateRobotModel();
        }
    }

    void ParseURDF(string urdfText)
    {
        XmlDocument doc = new XmlDocument();
        doc.LoadXml(urdfText);

        // Parse links
        XmlNodeList linkNodes = doc.SelectNodes("//link");
        foreach (XmlNode linkNode in linkNodes)
        {
            LinkData link = new LinkData();
            link.name = linkNode.Attributes["name"].Value;

            // Parse visual element
            XmlNode visualNode = linkNode.SelectSingleNode("visual");
            if (visualNode != null)
            {
                XmlNode geometryNode = visualNode.SelectSingleNode("geometry");
                if (geometryNode != null)
                {
                    if (geometryNode.SelectSingleNode("box") != null)
                    {
                        link.geometryType = "box";
                        XmlNode sizeNode = geometryNode.SelectSingleNode("box").Attributes["size"];
                        string[] sizeValues = sizeNode.Value.Split(' ');
                        link.size = new Vector3(
                            float.Parse(sizeValues[0]),
                            float.Parse(sizeValues[1]),
                            float.Parse(sizeValues[2])
                        );
                    }
                    // Add other geometry types (cylinder, sphere, mesh)
                }
            }

            links.Add(link);
        }

        // Parse joints
        XmlNodeList jointNodes = doc.SelectNodes("//joint");
        foreach (XmlNode jointNode in jointNodes)
        {
            JointData joint = new JointData();
            joint.name = jointNode.Attributes["name"].Value;
            joint.type = jointNode.Attributes["type"].Value;

            XmlNode parent = jointNode.SelectSingleNode("parent");
            XmlNode child = jointNode.SelectSingleNode("child");

            if (parent != null && child != null)
            {
                joint.parentLink = parent.Attributes["link"].Value;
                joint.childLink = child.Attributes["link"].Value;
            }

            XmlNode axisNode = jointNode.SelectSingleNode("axis");
            if (axisNode != null)
            {
                XmlNode xyz = axisNode.Attributes["xyz"];
                string[] axisValues = xyz.Value.Split(' ');
                joint.axis = new Vector3(
                    float.Parse(axisValues[0]),
                    float.Parse(axisValues[1]),
                    float.Parse(axisValues[2])
                );
            }

            joints.Add(joint);
        }
    }

    void CreateRobotModel()
    {
        foreach (LinkData link in links)
        {
            GameObject linkGO = new GameObject(link.name);
            linkGO.transform.SetParent(transform);

            // Create appropriate primitive based on geometry type
            switch (link.geometryType)
            {
                case "box":
                    CreateBoxCollider(linkGO, link.size);
                    break;
                case "cylinder":
                    CreateCylinder(linkGO, link.size);
                    break;
                case "sphere":
                    CreateSphereCollider(linkGO, link.size.x / 2);
                    break;
                case "mesh":
                    // Load mesh from file
                    CreateMeshCollider(linkGO, link.meshFile);
                    break;
            }
        }

        // Create joints
        foreach (JointData joint in joints)
        {
            CreateJoint(joint);
        }
    }

    void CreateBoxCollider(GameObject go, Vector3 size)
    {
        BoxCollider collider = go.AddComponent<BoxCollider>();
        collider.size = size;

        // Add visual representation
        GameObject visual = GameObject.CreatePrimitive(PrimitiveType.Cube);
        visual.transform.SetParent(go.transform);
        visual.transform.localScale = size;
        Destroy(visual.GetComponent<BoxCollider>());
    }

    void CreateJoint(JointData jointData)
    {
        Transform parent = transform.Find(jointData.parentLink);
        Transform child = transform.Find(jointData.childLink);

        if (parent != null && child != null)
        {
            child.SetParent(parent);

            // Add appropriate joint component based on joint type
            switch (jointData.type)
            {
                case "revolute":
                case "continuous":
                    var hingeJoint = child.gameObject.AddComponent<HingeJoint>();
                    hingeJoint.axis = jointData.axis;
                    break;
                case "prismatic":
                    var fixedJoint = child.gameObject.AddComponent<ConfigurableJoint>();
                    // Configure for prismatic motion
                    break;
                case "fixed":
                    // No additional joint needed, already parented
                    break;
            }
        }
    }
}
```

## Sensor Integration

### Publishing Sensor Data from Unity

```csharp
using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using Unity.Robotics.ROSTCPConnector.MessageTypes.Sensor;
using Unity.Robotics.ROSTCPConnector.MessageTypes.Geometry;

public class SensorPublisher : MonoBehaviour
{
    [Header("Sensor Topics")]
    public string lidarTopic = "/scan";
    public string imuTopic = "/imu/data";
    public string cameraTopic = "/camera/image_raw";

    [Header("Sensor Settings")]
    public float publishRate = 30.0f; // Hz

    private float publishInterval;
    private float lastPublishTime;

    void Start()
    {
        publishInterval = 1.0f / publishRate;
        lastPublishTime = Time.time;
    }

    void Update()
    {
        if (Time.time - lastPublishTime >= publishInterval)
        {
            PublishSensorData();
            lastPublishTime = Time.time;
        }
    }

    void PublishSensorData()
    {
        var ros = ROSConnection.GetOrCreateInstance();

        // Publish IMU data
        PublishIMUData(ros);

        // Publish other sensor data as needed
    }

    void PublishIMUData(ROSConnection ros)
    {
        var imuMsg = new ImuMsg();

        // Set orientation (from Unity's transform)
        Quaternion unityRot = transform.rotation;
        imuMsg.orientation.x = unityRot.x;
        imuMsg.orientation.y = unityRot.z;  // Unity Y-up to ROS Z-up
        imuMsg.orientation.z = unityRot.y;
        imuMsg.orientation.w = unityRot.w;

        // Set angular velocity
        imuMsg.angular_velocity.x = 0.0; // Fill with actual values
        imuMsg.angular_velocity.y = 0.0;
        imuMsg.angular_velocity.z = 0.0;

        // Set linear acceleration
        imuMsg.linear_acceleration.x = 0.0; // Fill with actual values
        imuMsg.linear_acceleration.y = 0.0;
        imuMsg.linear_acceleration.z = 0.0;

        // Publish the message
        ros.Publish(imuTopic, imuMsg);
    }
}
```

## Performance Optimization

### Efficient Data Transmission

Minimize network overhead in Unity-Gazebo communication:

```csharp
using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using System.Collections.Generic;

public class OptimizedDataTransmitter : MonoBehaviour
{
    [Header("Optimization Settings")]
    public float transformSyncThreshold = 0.01f; // Only sync if position changed significantly
    public int maxMessagesPerFrame = 10; // Limit messages per frame

    private Dictionary<string, Vector3> lastSyncedPositions = new Dictionary<string, Vector3>();
    private Dictionary<string, Quaternion> lastSyncedRotations = new Dictionary<string, Quaternion>();

    void Update()
    {
        SyncChangedTransforms();
    }

    void SyncChangedTransforms()
    {
        int messageCount = 0;
        var ros = ROSConnection.GetOrCreateInstance();

        foreach (Transform child in transform)
        {
            if (messageCount >= maxMessagesPerFrame)
                break;

            string modelName = child.name;

            Vector3 currentPosition = child.position;
            Quaternion currentRotation = child.rotation;

            Vector3 lastPosition = Vector3.zero;
            Quaternion lastRotation = Quaternion.identity;

            bool positionExists = lastSyncedPositions.TryGetValue(modelName, out lastPosition);
            bool rotationExists = lastSyncedRotations.TryGetValue(modelName, out lastRotation);

            bool needsSync = false;

            if (!positionExists || Vector3.Distance(currentPosition, lastPosition) > transformSyncThreshold)
            {
                needsSync = true;
                lastSyncedPositions[modelName] = currentPosition;
            }

            if (!rotationExists || Quaternion.Angle(currentRotation, lastRotation) > transformSyncThreshold * 10)
            {
                needsSync = true;
                lastSyncedRotations[modelName] = currentRotation;
            }

            if (needsSync)
            {
                PublishTransformUpdate(ros, modelName, currentPosition, currentRotation);
                messageCount++;
            }
        }
    }

    void PublishTransformUpdate(ROSConnection ros, string modelName, Vector3 position, Quaternion rotation)
    {
        // Publish transform update to Gazebo
        // Implementation depends on specific message format
    }
}
```

### Threading Considerations

Handle ROS communication efficiently:

```csharp
using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using System.Threading.Tasks;

public class ThreadedROSHandler : MonoBehaviour
{
    private ROSConnection ros;
    private bool shouldStop = false;

    void Start()
    {
        ros = ROSConnection.GetOrCreateInstance();
        StartBackgroundProcessing();
    }

    void StartBackgroundProcessing()
    {
        Task.Run(async () =>
        {
            while (!shouldStop)
            {
                // Process ROS messages in background thread
                // Be careful with Unity object access from background threads
                await Task.Delay(10); // 100 Hz processing
            }
        });
    }

    void OnDestroy()
    {
        shouldStop = true;
    }
}
```

## Troubleshooting Integration Issues

### Common Problems and Solutions

#### 1. Coordinate System Mismatch

**Problem**: Objects appear in wrong positions or orientations.

**Solution**: Implement proper coordinate conversion:

```csharp
public static class CoordinateConverter
{
    // Convert from ROS (right-handed) to Unity (left-handed)
    public static Vector3 RosToUnityPosition(Vector3 rosPos)
    {
        return new Vector3(rosPos.x, rosPos.z, rosPos.y);
    }

    public static Quaternion RosToUnityRotation(Quaternion rosRot)
    {
        return new Quaternion(rosRot.x, rosRot.z, rosRot.y, rosRot.w);
    }

    // Convert from Unity to ROS
    public static Vector3 UnityToRosPosition(Vector3 unityPos)
    {
        return new Vector3(unityPos.x, unityPos.z, unityPos.y);
    }
}
```

#### 2. Network Connection Issues

**Problem**: Unity cannot connect to ROS bridge.

**Solutions**:
- Check firewall settings
- Verify ROS bridge is running on correct port
- Ensure ROS_DOMAIN_ID matches between systems
- Check network connectivity with `telnet localhost 9090`

#### 3. Performance Issues

**Problem**: Low frame rate or network lag.

**Solutions**:
- Reduce sync rate for non-critical objects
- Implement level of detail (LOD) for distant objects
- Use efficient data structures
- Consider message throttling

## Best Practices

### 1. Modular Design

Create reusable components for different robot types:

```csharp
public abstract class RobotIntegrationBase : MonoBehaviour
{
    protected virtual void InitializeROSConnection() { }
    protected virtual void SubscribeToTopics() { }
    protected virtual void PublishToTopics() { }
    protected virtual void SyncRobotState() { }
}
```

### 2. Error Handling

Implement robust error handling:

```csharp
public class RobustROSConnection : MonoBehaviour
{
    [Header("Connection Settings")]
    public int maxRetries = 5;
    public float retryDelay = 1.0f;

    private int retryCount = 0;

    void OnConnectionError()
    {
        if (retryCount < maxRetries)
        {
            Invoke("Reconnect", retryDelay);
            retryCount++;
        }
        else
        {
            Debug.LogError("Max connection retries exceeded");
        }
    }

    void Reconnect()
    {
        // Attempt to reestablish connection
    }
}
```

## Summary

Unity-Gazebo integration requires careful coordination between physics simulation and visual rendering. By understanding communication patterns, synchronization techniques, and performance optimization strategies, you can create robust digital twin environments that accurately represent robotic systems.

## Next Steps

Now that you understand Unity-Gazebo integration techniques, proceed to [Interaction Design](./interaction-design.md) to learn how to create effective human-robot interaction visualizations.