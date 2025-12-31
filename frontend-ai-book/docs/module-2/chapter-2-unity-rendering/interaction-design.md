---
sidebar_position: 8
title: "Human-Robot Interaction Design"
---

# Human-Robot Interaction Design in Unity

## Overview

This section focuses on designing effective visual interfaces for human-robot interaction in Unity. You'll learn to create intuitive visualizations that enhance understanding, control, and communication between humans and robots in simulation environments.

## Principles of Human-Robot Interaction Visualization

### 1. Intuitive Representation

Effective HRI visualization should make robot behavior and state immediately understandable:

- **Clear affordances**: Visual cues that suggest how to interact
- **Consistent mapping**: Relationships between controls and robot actions
- **Feedback visibility**: Clear indication of robot responses to human input

### 2. State Communication

Robots must clearly communicate their current state to human operators:

```csharp
using UnityEngine;
using UnityEngine.UI;

public class RobotStateVisualizer : MonoBehaviour
{
    [Header("State Indicators")]
    public Light statusLight;
    public Text stateText;
    public Slider batteryLevel;
    public Image statusIcon;

    [Header("State Colors")]
    public Color idleColor = Color.blue;
    public Color activeColor = Color.green;
    public Color warningColor = Color.yellow;
    public Color errorColor = Color.red;

    public enum RobotState
    {
        Idle,
        Active,
        Warning,
        Error,
        Charging
    }

    private RobotState currentState = RobotState.Idle;
    private float batteryPercentage = 100.0f;

    void Update()
    {
        UpdateVisualIndicators();
    }

    void UpdateVisualIndicators()
    {
        // Update status light color
        switch (currentState)
        {
            case RobotState.Idle:
                statusLight.color = idleColor;
                break;
            case RobotState.Active:
                statusLight.color = activeColor;
                break;
            case RobotState.Warning:
                statusLight.color = warningColor;
                statusLight.intensity = Mathf.PingPong(Time.time * 2, 1) + 0.5f; // Pulsing
                break;
            case RobotState.Error:
                statusLight.color = errorColor;
                statusLight.intensity = Mathf.PingPong(Time.time * 4, 1) + 0.5f; // Fast pulsing
                break;
            case RobotState.Charging:
                statusLight.color = Color.cyan;
                statusLight.intensity = 0.5f + Mathf.Sin(Time.time * 5) * 0.3f; // Pulsing
                break;
        }

        // Update state text
        stateText.text = currentState.ToString();
        stateText.color = GetStateTextColor(currentState);

        // Update battery level
        batteryLevel.value = batteryPercentage;
        batteryLevel.fillRect.GetComponent<Image>().color = GetBatteryColor(batteryPercentage);
    }

    Color GetStateTextColor(RobotState state)
    {
        switch (state)
        {
            case RobotState.Error:
                return errorColor;
            case RobotState.Warning:
                return warningColor;
            default:
                return Color.white;
        }
    }

    Color GetBatteryColor(float percentage)
    {
        if (percentage > 50) return Color.green;
        else if (percentage > 20) return Color.yellow;
        else return Color.red;
    }

    public void SetRobotState(RobotState newState)
    {
        currentState = newState;
    }

    public void SetBatteryLevel(float percentage)
    {
        batteryPercentage = Mathf.Clamp(percentage, 0, 100);
    }
}
```

### 3. Intention Communication

Robots should clearly indicate their intentions and planned actions:

```csharp
using UnityEngine;
using System.Collections.Generic;

public class RobotIntentionVisualizer : MonoBehaviour
{
    [Header("Intention Visualization")]
    public Color pathColor = Color.green;
    public Color goalColor = Color.blue;
    public Color obstacleColor = Color.red;
    public float pathWidth = 0.1f;

    [Header("Visualization Settings")]
    public bool showPath = true;
    public bool showGoal = true;
    public bool showObstacles = true;

    private List<Vector3> plannedPath = new List<Vector3>();
    private Vector3 goalPosition;
    private List<Vector3> obstaclePositions = new List<Vector3>();

    void OnRenderObject()
    {
        if (showPath && plannedPath.Count > 1)
        {
            DrawPath();
        }

        if (showGoal)
        {
            DrawGoal();
        }

        if (showObstacles)
        {
            DrawObstacles();
        }
    }

    void DrawPath()
    {
        GL.Begin(GL.LINES);
        GL.Color(pathColor);
        GL.LineWidth(pathWidth);

        for (int i = 0; i < plannedPath.Count - 1; i++)
        {
            GL.Vertex(plannedPath[i]);
            GL.Vertex(plannedPath[i + 1]);
        }

        GL.End();
    }

    void DrawGoal()
    {
        GL.Begin(GL.QUADS);
        GL.Color(goalColor);

        Vector3 size = Vector3.one * 0.5f;
        Vector3 center = goalPosition;

        // Draw a simple cube at goal position
        GL.Vertex(center + new Vector3(-size.x, -size.y, -size.z));
        GL.Vertex(center + new Vector3(size.x, -size.y, -size.z));
        GL.Vertex(center + new Vector3(size.x, size.y, -size.z));
        GL.Vertex(center + new Vector3(-size.x, size.y, -size.z));

        GL.End();
    }

    void DrawObstacles()
    {
        GL.Begin(GL.LINES);
        GL.Color(obstacleColor);

        foreach (Vector3 obstaclePos in obstaclePositions)
        {
            // Draw a simple wireframe cube at obstacle position
            Vector3 size = Vector3.one * 0.3f;
            DrawWireframeCube(obstaclePos, size);
        }

        GL.End();
    }

    void DrawWireframeCube(Vector3 center, Vector3 size)
    {
        Vector3[] corners = new Vector3[8];

        corners[0] = center + new Vector3(-size.x, -size.y, -size.z);
        corners[1] = center + new Vector3(size.x, -size.y, -size.z);
        corners[2] = center + new Vector3(size.x, size.y, -size.z);
        corners[3] = center + new Vector3(-size.x, size.y, -size.z);
        corners[4] = center + new Vector3(-size.x, -size.y, size.z);
        corners[5] = center + new Vector3(size.x, -size.y, size.z);
        corners[6] = center + new Vector3(size.x, size.y, size.z);
        corners[7] = center + new Vector3(-size.x, size.y, size.z);

        // Draw edges
        int[,] edges = new int[,] {
            {0,1}, {1,2}, {2,3}, {3,0}, // Bottom face
            {4,5}, {5,6}, {6,7}, {7,4}, // Top face
            {0,4}, {1,5}, {2,6}, {3,7}  // Vertical edges
        };

        for (int i = 0; i < edges.GetLength(0); i++)
        {
            GL.Vertex(corners[edges[i, 0]]);
            GL.Vertex(corners[edges[i, 1]]);
        }
    }

    public void SetPlannedPath(List<Vector3> path)
    {
        plannedPath = path;
    }

    public void SetGoalPosition(Vector3 goal)
    {
        goalPosition = goal;
    }

    public void SetObstaclePositions(List<Vector3> obstacles)
    {
        obstaclePositions = obstacles;
    }
}
```

## Interaction Modalities

### 1. Direct Manipulation

Allow users to directly interact with robot components:

```csharp
using UnityEngine;

public class DirectManipulationController : MonoBehaviour
{
    [Header("Manipulation Settings")]
    public float manipulationDistance = 5.0f;
    public LayerMask robotLayer;
    public Camera interactionCamera;

    private GameObject selectedObject;
    private bool isDragging = false;
    private Vector3 offset;

    void Update()
    {
        HandleMouseInput();
    }

    void HandleMouseInput()
    {
        if (Input.GetMouseButtonDown(0))
        {
            Ray ray = interactionCamera.ScreenPointToRay(Input.mousePosition);
            RaycastHit hit;

            if (Physics.Raycast(ray, out hit, manipulationDistance, robotLayer))
            {
                selectedObject = hit.collider.gameObject;
                isDragging = true;
                offset = selectedObject.transform.position - ray.GetPoint(hit.distance);
            }
        }

        if (Input.GetMouseButtonUp(0))
        {
            isDragging = false;
            selectedObject = null;
        }

        if (isDragging && selectedObject != null)
        {
            Ray ray = interactionCamera.ScreenPointToRay(Input.mousePosition);
            Vector3 targetPosition = ray.GetPoint(hit.distance) + offset;
            selectedObject.transform.position = targetPosition;
        }
    }

    void OnGUI()
    {
        if (selectedObject != null)
        {
            // Display manipulation info
            Vector3 screenPos = interactionCamera.WorldToScreenPoint(selectedObject.transform.position);
            GUI.Label(new Rect(screenPos.x, Screen.height - screenPos.y, 200, 20),
                     "Manipulating: " + selectedObject.name);
        }
    }
}
```

### 2. Gesture-Based Interaction

Implement gesture recognition for more natural interaction:

```csharp
using UnityEngine;
using System.Collections.Generic;

public class GestureBasedInteraction : MonoBehaviour
{
    [Header("Gesture Settings")]
    public float gestureThreshold = 0.1f;
    public float gestureTimeout = 2.0f;

    private List<Vector3> gesturePoints = new List<Vector3>();
    private float gestureStartTime;
    private bool recordingGesture = false;

    void Update()
    {
        HandleGestureInput();
    }

    void HandleGestureInput()
    {
        if (Input.GetMouseButtonDown(0))
        {
            StartRecordingGesture();
        }
        else if (Input.GetMouseButton(0))
        {
            RecordGesturePoint();
        }
        else if (Input.GetMouseButtonUp(0))
        {
            ProcessGesture();
        }

        // Check for timeout
        if (recordingGesture && Time.time - gestureStartTime > gestureTimeout)
        {
            ResetGesture();
        }
    }

    void StartRecordingGesture()
    {
        gesturePoints.Clear();
        gestureStartTime = Time.time;
        recordingGesture = true;
    }

    void RecordGesturePoint()
    {
        if (recordingGesture)
        {
            Vector3 mousePos = Input.mousePosition;
            if (gesturePoints.Count == 0 ||
                Vector3.Distance(mousePos, gesturePoints[gesturePoints.Count - 1]) > gestureThreshold)
            {
                gesturePoints.Add(mousePos);
            }
        }
    }

    void ProcessGesture()
    {
        if (recordingGesture && gesturePoints.Count > 3)
        {
            RecognizeGesture(gesturePoints);
        }
        ResetGesture();
    }

    void RecognizeGesture(List<Vector3> points)
    {
        // Simple gesture recognition based on point patterns
        Vector3 start = points[0];
        Vector3 end = points[points.Count - 1];
        Vector3 direction = (end - start).normalized;

        // Recognize simple directional gestures
        if (Mathf.Abs(direction.x) > Mathf.Abs(direction.y))
        {
            if (direction.x > 0)
            {
                Debug.Log("Right swipe gesture detected");
                // Move robot right
            }
            else
            {
                Debug.Log("Left swipe gesture detected");
                // Move robot left
            }
        }
        else
        {
            if (direction.y > 0)
            {
                Debug.Log("Up swipe gesture detected");
                // Move robot up/forward
            }
            else
            {
                Debug.Log("Down swipe gesture detected");
                // Move robot down/backward
            }
        }
    }

    void ResetGesture()
    {
        recordingGesture = false;
        gesturePoints.Clear();
    }
}
```

### 3. Voice Command Integration

Integrate voice commands for hands-free interaction:

```csharp
using UnityEngine;

public class VoiceCommandHandler : MonoBehaviour
{
    [Header("Voice Commands")]
    public string[] moveForwardCommands = {"forward", "go forward", "move forward"};
    public string[] moveBackwardCommands = {"backward", "go back", "move back"};
    public string[] turnLeftCommands = {"turn left", "left", "rotate left"};
    public string[] turnRightCommands = {"turn right", "right", "rotate right"};
    public string[] stopCommands = {"stop", "halt", "pause"};

    [Header("Command Settings")]
    public float commandTimeout = 5.0f;

    private bool listeningForCommand = false;
    private float commandStartTime;

    void Start()
    {
        StartListeningForCommands();
    }

    void StartListeningForCommands()
    {
        // In a real implementation, this would connect to a speech recognition API
        // For simulation, we'll use keyboard input as a proxy
        Debug.Log("Voice command system initialized. Press spacebar to simulate voice input.");
    }

    void Update()
    {
        // Simulate voice command input with keyboard
        if (Input.GetKeyDown(KeyCode.Space))
        {
            SimulateVoiceCommand();
        }

        // Handle command timeout
        if (listeningForCommand && Time.time - commandStartTime > commandTimeout)
        {
            Debug.Log("Command timeout. Listening stopped.");
            listeningForCommand = false;
        }
    }

    void SimulateVoiceCommand()
    {
        // In a real implementation, this would receive actual voice input
        // For simulation, we'll randomly select a command
        string[] allCommands = new string[moveForwardCommands.Length +
                                         moveBackwardCommands.Length +
                                         turnLeftCommands.Length +
                                         turnRightCommands.Length +
                                         stopCommands.Length];

        int index = 0;
        System.Array.Copy(moveForwardCommands, 0, allCommands, index, moveForwardCommands.Length);
        index += moveForwardCommands.Length;
        System.Array.Copy(moveBackwardCommands, 0, allCommands, index, moveBackwardCommands.Length);
        index += moveBackwardCommands.Length;
        System.Array.Copy(turnLeftCommands, 0, allCommands, index, turnLeftCommands.Length);
        index += turnLeftCommands.Length;
        System.Array.Copy(turnRightCommands, 0, allCommands, index, turnRightCommands.Length);
        index += turnRightCommands.Length;
        System.Array.Copy(stopCommands, 0, allCommands, index, stopCommands.Length);

        string randomCommand = allCommands[Random.Range(0, allCommands.Length)];
        Debug.Log($"Simulated voice command: '{randomCommand}'");

        ProcessVoiceCommand(randomCommand);
    }

    void ProcessVoiceCommand(string command)
    {
        command = command.ToLower().Trim();

        if (IsCommandInArray(command, moveForwardCommands))
        {
            Debug.Log("Moving robot forward");
            // Send move forward command to robot
        }
        else if (IsCommandInArray(command, moveBackwardCommands))
        {
            Debug.Log("Moving robot backward");
            // Send move backward command to robot
        }
        else if (IsCommandInArray(command, turnLeftCommands))
        {
            Debug.Log("Turning robot left");
            // Send turn left command to robot
        }
        else if (IsCommandInArray(command, turnRightCommands))
        {
            Debug.Log("Turning robot right");
            // Send turn right command to robot
        }
        else if (IsCommandInArray(command, stopCommands))
        {
            Debug.Log("Stopping robot");
            // Send stop command to robot
        }
        else
        {
            Debug.Log($"Unknown command: '{command}'");
        }
    }

    bool IsCommandInArray(string command, string[] commandArray)
    {
        foreach (string cmd in commandArray)
        {
            if (command.Contains(cmd.ToLower()))
            {
                return true;
            }
        }
        return false;
    }
}
```

## Visualization Techniques

### 1. Overlay Visualization

Create information overlays that don't obstruct the main view:

```csharp
using UnityEngine;
using UnityEngine.UI;

public class OverlayVisualizer : MonoBehaviour
{
    [Header("Overlay Settings")]
    public GameObject overlayPanel;
    public Text robotInfoText;
    public Slider progressSlider;
    public Image progressBarFill;
    public Button[] controlButtons;

    [Header("Visual Settings")]
    public Color overlayBackgroundColor = new Color(0, 0, 0, 0.5f);
    public Color progressBarColor = Color.green;

    private float currentProgress = 0.0f;
    private bool overlayVisible = true;

    void Start()
    {
        SetupOverlay();
    }

    void SetupOverlay()
    {
        // Configure overlay panel
        Image panelImage = overlayPanel.GetComponent<Image>();
        panelImage.color = overlayBackgroundColor;

        // Configure progress bar
        Image fillImage = progressBarFill;
        fillImage.color = progressBarColor;

        // Add button listeners
        foreach (Button button in controlButtons)
        {
            button.onClick.AddListener(() => OnControlButtonClicked(button.name));
        }
    }

    void Update()
    {
        UpdateOverlayInfo();
        ToggleOverlayVisibility();
    }

    void UpdateOverlayInfo()
    {
        // Update robot information
        robotInfoText.text = $"Robot Status: Active\nBattery: 85%\nPosition: ({transform.position.x:F2}, {transform.position.z:F2})";

        // Update progress
        progressSlider.value = currentProgress;
    }

    void ToggleOverlayVisibility()
    {
        if (Input.GetKeyDown(KeyCode.Tab))
        {
            overlayVisible = !overlayVisible;
            overlayPanel.SetActive(overlayVisible);
        }
    }

    void OnControlButtonClicked(string buttonName)
    {
        Debug.Log($"Control button clicked: {buttonName}");
        // Handle control button actions
    }

    public void SetProgress(float progress)
    {
        currentProgress = Mathf.Clamp01(progress);
    }

    public void SetRobotInfo(string info)
    {
        robotInfoText.text = info;
    }
}
```

### 2. Augmented Reality Elements

Implement AR-style elements for enhanced visualization:

```csharp
using UnityEngine;

public class ARVisualizer : MonoBehaviour
{
    [Header("AR Elements")]
    public GameObject[] arElements;
    public LineRenderer[] connectionLines;
    public GameObject infoTagsParent;

    [Header("AR Settings")]
    public float tagDistance = 2.0f;
    public Color connectionColor = Color.cyan;
    public float connectionWidth = 0.05f;

    void Start()
    {
        InitializeARElements();
    }

    void InitializeARElements()
    {
        foreach (GameObject element in arElements)
        {
            SetupARForObject(element);
        }
    }

    void SetupARForObject(GameObject targetObject)
    {
        // Create info tag above object
        GameObject tag = new GameObject(targetObject.name + "_Tag");
        tag.transform.SetParent(infoTagsParent.transform);

        // Position tag above the object
        tag.transform.position = targetObject.transform.position + Vector3.up * tagDistance;

        // Add UI components to tag
        SetupTagComponents(tag, targetObject.name);
    }

    void SetupTagComponents(GameObject tag, string objectName)
    {
        // Add text component
        TextMesh textMesh = tag.AddComponent<TextMesh>();
        textMesh.text = objectName;
        textMesh.fontSize = 20;
        textMesh.anchor = TextAnchor.MiddleCenter;
        textMesh.color = Color.white;

        // Add background
        GameObject background = GameObject.CreatePrimitive(PrimitiveType.Quad);
        background.transform.SetParent(tag.transform);
        background.transform.localPosition = Vector3.zero;
        background.transform.localRotation = Quaternion.Euler(90, 0, 0); // Face camera
        background.transform.localScale = new Vector3(0.5f, 0.1f, 1);

        // Set background material
        Renderer bgRenderer = background.GetComponent<Renderer>();
        bgRenderer.material = new Material(Shader.Find("Sprites/Default"));
        bgRenderer.material.color = new Color(0, 0, 0, 0.7f);
    }

    void Update()
    {
        UpdateARElements();
    }

    void UpdateARElements()
    {
        // Update positions to follow their targets
        for (int i = 0; i < arElements.Length; i++)
        {
            if (i < infoTagsParent.transform.childCount)
            {
                Transform tag = infoTagsParent.transform.GetChild(i);
                tag.position = arElements[i].transform.position + Vector3.up * tagDistance;

                // Make tag face the camera
                tag.LookAt(Camera.main.transform);
                tag.Rotate(0, 180, 0); // Correct for Unity's coordinate system
            }
        }

        // Update connection lines
        UpdateConnectionLines();
    }

    void UpdateConnectionLines()
    {
        for (int i = 0; i < connectionLines.Length && i < arElements.Length - 1; i++)
        {
            if (connectionLines[i] != null && arElements[i] != null && arElements[i + 1] != null)
            {
                connectionLines[i].SetPosition(0, arElements[i].transform.position);
                connectionLines[i].SetPosition(1, arElements[i + 1].transform.position);
            }
        }
    }

    public void AddARForObject(GameObject newObject)
    {
        GameObject[] newElements = new GameObject[arElements.Length + 1];
        System.Array.Copy(arElements, newElements, arElements.Length);
        newElements[newElements.Length - 1] = newObject;
        arElements = newElements;

        SetupARForObject(newObject);
    }
}
```

## Safety and Feedback Systems

### 1. Safety Zone Visualization

Visualize safety zones around the robot:

```csharp
using UnityEngine;
using System.Collections.Generic;

public class SafetyZoneVisualizer : MonoBehaviour
{
    [Header("Safety Zones")]
    public float personalSpaceRadius = 1.0f;
    public float warningZoneRadius = 2.0f;
    public float dangerZoneRadius = 0.5f;

    [Header("Zone Colors")]
    public Color dangerColor = new Color(1, 0, 0, 0.3f);
    public Color warningColor = new Color(1, 1, 0, 0.2f);
    public Color personalSpaceColor = new Color(0, 0, 1, 0.1f);

    private GameObject dangerZone;
    private GameObject warningZone;
    private GameObject personalSpaceZone;

    void Start()
    {
        CreateSafetyZones();
    }

    void CreateSafetyZones()
    {
        // Create danger zone (closest to robot)
        dangerZone = CreateSemiTransparentSphere(dangerZoneRadius, dangerColor);
        dangerZone.name = "DangerZone";
        dangerZone.transform.SetParent(transform, false);

        // Create warning zone
        warningZone = CreateSemiTransparentSphere(warningZoneRadius, warningColor);
        warningZone.name = "WarningZone";
        warningZone.transform.SetParent(transform, false);

        // Create personal space zone (outermost)
        personalSpaceZone = CreateSemiTransparentSphere(personalSpaceRadius, personalSpaceColor);
        personalSpaceZone.name = "PersonalSpaceZone";
        personalSpaceZone.transform.SetParent(transform, false);
    }

    GameObject CreateSemiTransparentSphere(float radius, Color color)
    {
        GameObject sphere = GameObject.CreatePrimitive(PrimitiveType.Sphere);
        sphere.transform.localScale = Vector3.one * radius * 2; // Primitive has radius of 0.5

        Renderer renderer = sphere.GetComponent<Renderer>();
        Material material = new Material(Shader.Find("Sprites/Default"));
        material.color = color;
        material.SetFloat("_Mode", 3); // Transparent mode
        material.SetInt("_SrcBlend", (int)UnityEngine.Rendering.BlendMode.SrcAlpha);
        material.SetInt("_DstBlend", (int)UnityEngine.Rendering.BlendMode.OneMinusSrcAlpha);
        material.SetInt("_ZWrite", 0);
        material.DisableKeyword("_ALPHATEST_ON");
        material.EnableKeyword("_ALPHABLEND_ON");
        material.DisableKeyword("_ALPHAPREMULTIPLY_ON");
        material.renderQueue = 3000;

        renderer.material = material;

        // Remove collider since we just want visualization
        Destroy(sphere.GetComponent<SphereCollider>());

        return sphere;
    }

    void Update()
    {
        UpdateZoneVisibility();
    }

    void UpdateZoneVisibility()
    {
        // Show/hide zones based on context
        bool showZones = true; // Could be based on user preference or robot state

        dangerZone.SetActive(showZones);
        warningZone.SetActive(showZones);
        personalSpaceZone.SetActive(showZones);
    }

    public void SetZoneRadii(float danger, float warning, float personal)
    {
        if (dangerZone != null)
            dangerZone.transform.localScale = Vector3.one * danger * 2;
        if (warningZone != null)
            warningZone.transform.localScale = Vector3.one * warning * 2;
        if (personalSpaceZone != null)
            personalSpaceZone.transform.localScale = Vector3.one * personal * 2;
    }
}
```

### 2. Feedback Systems

Implement comprehensive feedback systems:

```csharp
using UnityEngine;
using UnityEngine.UI;
using System.Collections;

public class FeedbackSystem : MonoBehaviour
{
    [Header("Feedback Components")]
    public Text feedbackText;
    public Image feedbackIcon;
    public Animator feedbackAnimator;
    public AudioSource feedbackAudio;

    [Header("Feedback Settings")]
    public float feedbackDuration = 3.0f;
    public AnimationCurve feedbackFadeCurve;

    private Queue<FeedbackMessage> feedbackQueue = new Queue<FeedbackMessage>();
    private bool isDisplayingFeedback = false;

    [System.Serializable]
    public class FeedbackMessage
    {
        public string text;
        public Sprite icon;
        public AudioClip audioClip;
        public FeedbackType type;
        public float duration;

        public enum FeedbackType
        {
            Information,
            Success,
            Warning,
            Error
        }
    }

    void Update()
    {
        if (!isDisplayingFeedback && feedbackQueue.Count > 0)
        {
            DisplayNextFeedback();
        }
    }

    public void AddFeedback(string text, FeedbackMessage.FeedbackType type, float duration = -1)
    {
        FeedbackMessage msg = new FeedbackMessage();
        msg.text = text;
        msg.type = type;
        msg.duration = duration > 0 ? duration : feedbackDuration;

        // Set icon based on type
        msg.icon = GetIconForType(type);

        // Set audio based on type
        msg.audioClip = GetAudioForType(type);

        feedbackQueue.Enqueue(msg);
    }

    Sprite GetIconForType(FeedbackMessage.FeedbackType type)
    {
        // In a real implementation, this would return appropriate icons
        // For now, return null or a default icon
        return null;
    }

    AudioClip GetAudioForType(FeedbackMessage.FeedbackType type)
    {
        // In a real implementation, this would return appropriate audio clips
        // For now, return null
        return null;
    }

    void DisplayNextFeedback()
    {
        if (feedbackQueue.Count == 0) return;

        FeedbackMessage msg = feedbackQueue.Dequeue();
        StartCoroutine(DisplayFeedbackCoroutine(msg));
    }

    IEnumerator DisplayFeedbackCoroutine(FeedbackMessage msg)
    {
        isDisplayingFeedback = true;

        // Show feedback
        if (feedbackText != null)
        {
            feedbackText.text = msg.text;
        }

        if (feedbackIcon != null && msg.icon != null)
        {
            feedbackIcon.sprite = msg.icon;
        }

        if (feedbackAudio != null && msg.audioClip != null)
        {
            feedbackAudio.PlayOneShot(msg.audioClip);
        }

        if (feedbackAnimator != null)
        {
            feedbackAnimator.SetTrigger("Show");
        }

        // Wait for duration
        yield return new WaitForSeconds(msg.duration);

        // Hide feedback
        if (feedbackText != null)
        {
            feedbackText.text = "";
        }

        if (feedbackIcon != null)
        {
            feedbackIcon.sprite = null;
        }

        if (feedbackAnimator != null)
        {
            feedbackAnimator.SetTrigger("Hide");
        }

        isDisplayingFeedback = false;
    }

    // Convenience methods for different feedback types
    public void ShowInfo(string message, float duration = -1)
    {
        AddFeedback(message, FeedbackMessage.FeedbackType.Information, duration);
    }

    public void ShowSuccess(string message, float duration = -1)
    {
        AddFeedback(message, FeedbackMessage.FeedbackType.Success, duration);
    }

    public void ShowWarning(string message, float duration = -1)
    {
        AddFeedback(message, FeedbackMessage.FeedbackType.Warning, duration);
    }

    public void ShowError(string message, float duration = -1)
    {
        AddFeedback(message, FeedbackMessage.FeedbackType.Error, duration);
    }
}
```

## Accessibility Considerations

### Visual Accessibility

Make HRI interfaces accessible to users with visual impairments:

```csharp
using UnityEngine;
using UnityEngine.UI;

public class AccessibleHRI : MonoBehaviour
{
    [Header("Accessibility Settings")]
    public bool highContrastMode = false;
    public float minimumContrastRatio = 4.5f;
    public bool enableAudioFeedback = true;
    public bool enableVibrationFeedback = true;

    [Header("Color Settings")]
    public Color highContrastColor1 = Color.white;
    public Color highContrastColor2 = Color.black;
    public Color normalColor1 = Color.blue;
    public Color normalColor2 = Color.gray;

    private bool accessibilityMode = false;

    void Start()
    {
        ApplyAccessibilitySettings();
    }

    void ApplyAccessibilitySettings()
    {
        if (highContrastMode)
        {
            SetHighContrastColors();
        }
    }

    void SetHighContrastColors()
    {
        // Apply high contrast colors to all UI elements
        foreach (Image image in GetComponentsInChildren<Image>())
        {
            if (image.color.grayscale < 0.5f) // Dark colors
            {
                image.color = highContrastColor2;
            }
            else // Light colors
            {
                image.color = highContrastColor1;
            }
        }

        foreach (Text text in GetComponentsInChildren<Text>())
        {
            text.color = highContrastColor1;
            text.fontSize = Mathf.RoundToInt(text.fontSize * 1.2f); // Slightly larger text
        }
    }

    public void ToggleAccessibilityMode()
    {
        accessibilityMode = !accessibilityMode;
        ApplyAccessibilitySettings();
    }

    public void ToggleHighContrast()
    {
        highContrastMode = !highContrastMode;
        ApplyAccessibilitySettings();
    }

    public void ToggleAudioFeedback()
    {
        enableAudioFeedback = !enableAudioFeedback;
    }

    public void ToggleVibrationFeedback()
    {
        enableVibrationFeedback = !enableVibrationFeedback;
    }
}
```

## Summary

Effective human-robot interaction design in Unity requires careful consideration of visualization techniques, interaction modalities, and feedback systems. By implementing clear state communication, intuitive controls, and accessible interfaces, you can create compelling and effective HRI experiences that enhance both user experience and robot functionality.

## Next Steps

Now that you understand human-robot interaction design principles, you have completed Chapter 2 on High-Fidelity Unity Rendering. The next chapter will cover Sensor Simulation for LiDAR, depth cameras, and IMUs in Gazebo.