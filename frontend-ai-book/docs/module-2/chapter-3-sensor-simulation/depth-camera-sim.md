---
sidebar_position: 11
title: "Depth Camera Simulation"
---

# Depth Camera Simulation in Gazebo

## Overview

This section covers depth camera sensor simulation in Gazebo. You'll learn to configure realistic depth cameras, generate RGB-D data (color + depth), and work with depth camera messages that match real-world sensor characteristics.

## Depth Camera Fundamentals

### How Depth Cameras Work

Depth cameras capture both color (RGB) and depth information simultaneously. Common types include:

1. **Stereo Cameras**: Use two cameras to calculate depth via triangulation
2. **Time-of-Flight (ToF)**: Measure time for light to return to sensor
3. **Structured Light**: Project pattern and analyze deformation

### Depth Camera Applications

- **3D Reconstruction**: Create 3D models of environments
- **Object Recognition**: Identify objects using both color and shape
- **SLAM**: Simultaneous Localization and Mapping with RGB-D data
- **Human-Robot Interaction**: Gesture recognition and pose estimation

### Types of Depth Camera Data

1. **RGB Image**: Color information (red, green, blue channels)
2. **Depth Image**: Distance information (usually in millimeters)
3. **Point Cloud**: 3D coordinates derived from depth data
4. **Normal Map**: Surface orientation information

## Gazebo Depth Camera Implementation

### Depth Camera Sensor Plugin

Gazebo uses the `libgazebo_ros_openni_kinect.so` plugin to simulate depth cameras:

```xml
<sensor name="depth_camera" type="depth">
  <always_on>true</always_on>
  <update_rate>30</update_rate>
  <camera>
    <horizontal_fov>1.047</horizontal_fov> <!-- 60 degrees -->
    <image>
      <width>640</width>
      <height>480</height>
      <format>R8G8B8</format>
    </image>
    <clip>
      <near>0.1</near> <!-- 10cm minimum -->
      <far>10.0</far>  <!-- 10m maximum -->
    </clip>
  </camera>

  <plugin name="camera_controller" filename="libgazebo_ros_openni_kinect.so">
    <alwaysOn>true</alwaysOn>
    <updateRate>30.0</updateRate>
    <cameraName>camera</cameraName>
    <imageTopicName>rgb/image_raw</imageTopicName>
    <depthImageTopicName>depth/image_raw</depthImageTopicName>
    <pointCloudTopicName>depth/points</pointCloudTopicName>
    <cameraInfoTopicName>rgb/camera_info</cameraInfoTopicName>
    <depthImageCameraInfoTopicName>depth/camera_info</depthImageCameraInfoTopicName>
    <frameName>camera_depth_frame</frameName>
    <baseline>0.2</baseline>
    <distortion_k1>0.0</distortion_k1>
    <distortion_k2>0.0</distortion_k2>
    <distortion_k3>0.0</distortion_k3>
    <distortion_t1>0.0</distortion_t1>
    <distortion_t2>0.0</distortion_t2>
    <pointCloudCutoff>0.1</pointCloudCutoff>
    <pointCloudCutoffMax>10.0</pointCloudCutoffMax>
    <CxPrime>0</CxPrime>
    <Cx>0</Cx>
    <Cy>0</Cy>
    <focalLength>0</focalLength>
    <hackBaseline>0</hackBaseline>
  </plugin>
</sensor>
```

### Key Depth Camera Parameters

#### Camera Parameters
- **Horizontal FOV**: Field of view in radians
- **Image Resolution**: Width and height in pixels
- **Format**: Color format (R8G8B8, B8G8R8, etc.)
- **Clip Range**: Near and far clipping planes

#### Plugin Parameters
- **Topic Names**: ROS topic names for different data streams
- **Frame Name**: TF frame for camera
- **Distortion Coefficients**: Camera calibration parameters
- **Point Cloud Cutoff**: Min/max distances for point cloud generation

## Configuring Different Depth Cameras

### Kinect-style Configuration

```xml
<sensor name="kinect_camera" type="depth">
  <always_on>true</always_on>
  <update_rate>30</update_rate>

  <camera>
    <horizontal_fov>1.0123</horizontal_fov> <!-- 58 degrees -->
    <image>
      <width>640</width>
      <height>480</height>
      <format>R8G8B8</format>
    </image>
    <depth_camera>
      <output>depths</output>
    </depth_camera>
    <clip>
      <near>0.5</near> <!-- 50cm minimum -->
      <far>4.0</far>   <!-- 4m maximum -->
    </clip>
  </camera>

  <plugin name="kinect_camera_plugin" filename="libgazebo_ros_openni_kinect.so">
    <alwaysOn>true</alwaysOn>
    <updateRate>30.0</updateRate>
    <cameraName>kinect</cameraName>
    <imageTopicName>rgb/image_raw</imageTopicName>
    <depthImageTopicName>depth/image_raw</depthImageTopicName>
    <pointCloudTopicName>depth/points</pointCloudTopicName>
    <cameraInfoTopicName>rgb/camera_info</cameraInfoTopicName>
    <depthImageCameraInfoTopicName>depth/camera_info</depthImageCameraInfoTopicName>
    <frameName>kinect_rgb_optical_frame</frameName>
    <baseline>0.1</baseline>
    <distortion_k1>0.0</distortion_k1>
    <distortion_k2>0.0</distortion_k2>
    <distortion_k3>0.0</distortion_k3>
    <distortion_t1>0.0</distortion_t1>
    <distortion_t2>0.0</distortion_t2>
    <pointCloudCutoff>0.5</pointCloudCutoff>
    <pointCloudCutoffMax>4.0</pointCloudCutoffMax>
  </plugin>
</sensor>
```

### High-Resolution Depth Camera

```xml
<sensor name="high_res_depth_camera" type="depth">
  <always_on>true</always_on>
  <update_rate>15</update_rate> <!-- Lower rate for higher resolution -->

  <camera>
    <horizontal_fov>0.7854</horizontal_fov> <!-- 45 degrees -->
    <image>
      <width>1280</width>
      <height>720</height>
      <format>R8G8B8</format>
    </image>
    <clip>
      <near>0.2</near>
      <far>8.0</far>
    </clip>
  </camera>

  <plugin name="high_res_camera_plugin" filename="libgazebo_ros_openni_kinect.so">
    <alwaysOn>true</alwaysOn>
    <updateRate>15.0</updateRate>
    <cameraName>high_res_camera</cameraName>
    <imageTopicName>rgb/image_raw</imageTopicName>
    <depthImageTopicName>depth/image_raw</depthImageTopicName>
    <pointCloudTopicName>depth/points</pointCloudTopicName>
    <cameraInfoTopicName>rgb/camera_info</cameraInfoTopicName>
    <depthImageCameraInfoTopicName>depth/camera_info</depthImageCameraInfoTopicName>
    <frameName>high_res_camera_frame</frameName>
    <pointCloudCutoff>0.2</pointCloudCutoff>
    <pointCloudCutoffMax>8.0</pointCloudCutoffMax>
  </plugin>
</sensor>
```

## Depth Data Processing

### Understanding Depth Images

Depth images contain distance information in each pixel:

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import numpy as np
from cv_bridge import CvBridge

class DepthImageProcessor(Node):
    def __init__(self):
        super().__init__('depth_image_processor')

        self.bridge = CvBridge()

        # Subscribe to depth image
        self.depth_subscription = self.create_subscription(
            Image,
            '/camera/depth/image_raw',
            self.depth_image_callback,
            10)

        # Publisher for processed depth data
        self.processed_publisher = self.create_publisher(
            Image,
            '/camera/depth/processed',
            10)

    def depth_image_callback(self, msg):
        # Convert ROS Image message to OpenCV format
        try:
            # For depth images, use '32FC1' format (32-bit float, 1 channel)
            depth_cv = self.bridge.imgmsg_to_cv2(msg, '32FC1')

            # Process depth data
            processed_depth = self.process_depth_data(depth_cv)

            # Convert back to ROS Image message
            processed_msg = self.bridge.cv2_to_imgmsg(processed_depth, '32FC1')
            processed_msg.header = msg.header

            # Publish processed data
            self.processed_publisher.publish(processed_msg)

        except Exception as e:
            self.get_logger().error(f'Error processing depth image: {e}')

    def process_depth_data(self, depth_image):
        """Process depth image data"""

        # Remove invalid depth values (inf or nan)
        depth_image = np.nan_to_num(depth_image, nan=0.0, posinf=0.0, neginf=0.0)

        # Apply depth thresholding
        min_depth = 0.5  # 50cm
        max_depth = 4.0  # 4m
        depth_image = np.clip(depth_image, min_depth, max_depth)

        # Optional: Apply median filter to reduce noise
        import cv2
        depth_image = cv2.medianBlur(depth_image, 5)

        return depth_image

    def get_depth_at_pixel(self, depth_image, u, v):
        """Get depth value at specific pixel coordinates"""
        if 0 <= u < depth_image.shape[1] and 0 <= v < depth_image.shape[0]:
            return depth_image[v, u]
        else:
            return 0.0  # Invalid coordinates
```

### Converting Depth to Point Cloud

```python
import numpy as np
from sensor_msgs.msg import PointCloud2, PointField
import sensor_msgs.point_cloud2 as pc2
from std_msgs.msg import Header

class DepthToPointCloud:
    def __init__(self):
        # Camera intrinsic parameters (these should match your camera config)
        self.fx = 525.0  # Focal length x
        self.fy = 525.0  # Focal length y
        self.cx = 319.5  # Principal point x
        self.cy = 239.5  # Principal point y

    def depth_to_pointcloud(self, depth_image, camera_info=None):
        """Convert depth image to point cloud"""

        if camera_info:
            # Use camera info if provided
            self.fx = camera_info.K[0]
            self.fy = camera_info.K[4]
            self.cx = camera_info.K[2]
            self.cy = camera_info.K[5]

        height, width = depth_image.shape
        points = []

        for v in range(height):
            for u in range(width):
                z = depth_image[v, u]

                if z > 0:  # Valid depth
                    # Convert pixel coordinates to 3D world coordinates
                    x = (u - self.cx) * z / self.fx
                    y = (v - self.cy) * z / self.fy

                    points.append([x, y, z])

        return points

    def create_pointcloud2_msg(self, points, header):
        """Create PointCloud2 message from list of points"""

        fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1)
        ]

        # Create PointCloud2 message
        pc_msg = PointCloud2()
        pc_msg.header = header
        pc_msg.height = 1
        pc_msg.width = len(points)
        pc_msg.fields = fields
        pc_msg.is_bigendian = False
        pc_msg.is_dense = False
        pc_msg.point_step = 12  # 3 floats * 4 bytes each
        pc_msg.row_step = pc_msg.point_step * pc_msg.width

        # Pack points into binary data
        import struct
        data = []
        for point in points:
            data.extend(struct.pack('fff', *point))

        pc_msg.data = b''.join(data)

        return pc_msg
```

## RGB-D Data Processing

### Combining Color and Depth

```python
import cv2
import numpy as np

class RGBDProcessor:
    def __init__(self):
        self.color_image = None
        self.depth_image = None

    def process_rgbd_data(self, color_msg, depth_msg):
        """Process synchronized RGB and depth data"""

        # Convert ROS messages to OpenCV images
        bridge = CvBridge()
        color_cv = bridge.imgmsg_to_cv2(color_msg, 'bgr8')
        depth_cv = bridge.imgmsg_to_cv2(depth_msg, '32FC1')

        # Ensure images are the same size
        if color_cv.shape[:2] != depth_cv.shape:
            # Resize depth image to match color image
            depth_cv = cv2.resize(depth_cv, (color_cv.shape[1], color_cv.shape[0]))

        # Create RGB-D data structure
        rgbd_data = {
            'color': color_cv,
            'depth': depth_cv,
            'height': color_cv.shape[0],
            'width': color_cv.shape[1],
            'timestamp': color_msg.header.stamp
        }

        return rgbd_data

    def extract_rgbd_features(self, rgbd_data):
        """Extract features from RGB-D data"""

        color = rgbd_data['color']
        depth = rgbd_data['depth']

        # Calculate depth statistics
        valid_depths = depth[depth > 0]  # Only valid depth values
        depth_stats = {
            'mean': np.mean(valid_depths) if len(valid_depths) > 0 else 0,
            'std': np.std(valid_depths) if len(valid_depths) > 0 else 0,
            'min': np.min(valid_depths) if len(valid_depths) > 0 else 0,
            'max': np.max(valid_depths) if len(valid_depths) > 0 else 0
        }

        # Extract color features
        color_features = {
            'mean_color': np.mean(color, axis=(0, 1)),
            'std_color': np.std(color, axis=(0, 1))
        }

        # Find depth-based regions of interest
        regions = self.find_depth_regions(depth)

        return {
            'depth_stats': depth_stats,
            'color_features': color_features,
            'regions': regions
        }

    def find_depth_regions(self, depth_image):
        """Find regions based on depth values"""

        # Threshold to find objects at different depths
        near_mask = (depth_image > 0) & (depth_image <= 1.0)  # 0-1m
        mid_mask = (depth_image > 1.0) & (depth_image <= 3.0)  # 1-3m
        far_mask = (depth_image > 3.0) & (depth_image <= 5.0)  # 3-5m

        # Find contours for each depth region
        import cv2

        contours_near, _ = cv2.findContours(
            near_mask.astype(np.uint8),
            cv2.RETR_EXTERNAL,
            cv2.CHAIN_APPROX_SIMPLE
        )

        contours_mid, _ = cv2.findContours(
            mid_mask.astype(np.uint8),
            cv2.RETR_EXTERNAL,
            cv2.CHAIN_APPROX_SIMPLE
        )

        contours_far, _ = cv2.findContours(
            far_mask.astype(np.uint8),
            cv2.RETR_EXTERNAL,
            cv2.CHAIN_APPROX_SIMPLE
        )

        return {
            'near_contours': contours_near,
            'mid_contours': contours_mid,
            'far_contours': contours_far
        }
```

## Performance Optimization

### Efficient Depth Processing

```cpp
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

class OptimizedDepthProcessor
{
private:
    cv::Mat temp_depth_buffer_;
    cv::Mat processed_depth_buffer_;
    size_t last_width_, last_height_;
    bool buffer_initialized_;

public:
    OptimizedDepthProcessor() : last_width_(0), last_height_(0), buffer_initialized_(false) {}

    void processDepthImage(const sensor_msgs::msg::Image::SharedPtr depth_msg)
    {
        // Convert to OpenCV
        cv_bridge::CvImagePtr cv_ptr;
        try {
            cv_ptr = cv_bridge::toCvCopy(depth_msg, sensor_msgs::image_encodings::TYPE_32FC1);
        } catch (cv_bridge::Exception& e) {
            RCLCPP_ERROR(rclcpp::get_logger("depth_processor"),
                        "cv_bridge exception: %s", e.what());
            return;
        }

        // Ensure buffers are the right size
        if (!buffer_initialized_ ||
            cv_ptr->image.cols != last_width_ ||
            cv_ptr->image.rows != last_height_) {

            temp_depth_buffer_ = cv::Mat(cv_ptr->image.rows, cv_ptr->image.cols, CV_32F);
            processed_depth_buffer_ = cv::Mat(cv_ptr->image.rows, cv_ptr->image.cols, CV_32F);
            last_width_ = cv_ptr->image.cols;
            last_height_ = cv_ptr->image.rows;
            buffer_initialized_ = true;
        }

        // Copy and process efficiently
        cv_ptr->image.copyTo(temp_depth_buffer_);

        // Apply processing with in-place operations
        processDepthInPlace(temp_depth_buffer_);

        // The processed data is now in processed_depth_buffer_
    }

    void processDepthInPlace(cv::Mat& depth_image)
    {
        // Vectorized operations for efficiency
        cv::Mat valid_mask = (depth_image > 0.0f) & (depth_image < 10.0f);

        // Apply thresholding efficiently
        depth_image.setTo(0.0f, ~valid_mask);

        // Optional: Apply bilateral filter for noise reduction
        // This is more efficient than median blur for depth images
        cv::bilateralFilter(depth_image, processed_depth_buffer_, 5, 50, 50);
    }
};
```

### Memory-Efficient Point Cloud Generation

```cpp
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <sensor_msgs/msg/point_cloud2.hpp>

class EfficientPointCloudGenerator
{
private:
    // Pre-allocated point cloud for reuse
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr pre_allocated_cloud_;
    size_t max_points_;

public:
    EfficientPointCloudGenerator(size_t max_width = 1280, size_t max_height = 720)
        : max_points_(max_width * max_height)
    {
        pre_allocated_cloud_.reset(new pcl::PointCloud<pcl::PointXYZRGB>());
        pre_allocated_cloud_->points.reserve(max_points_);
    }

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr generatePointCloud(
        const cv::Mat& color_image,
        const cv::Mat& depth_image,
        const CameraIntrinsics& intrinsics)
    {
        // Clear previous points but keep allocated memory
        pre_allocated_cloud_->clear();
        pre_allocated_cloud_->width = 0;
        pre_allocated_cloud_->height = 1;
        pre_allocated_cloud_->is_dense = false;

        // Resize if necessary (shouldn't happen if images are consistent)
        if (color_image.size() != depth_image.size()) {
            // Handle size mismatch
            return pre_allocated_cloud_;
        }

        // Process pixels efficiently
        for (int v = 0; v < depth_image.rows; ++v) {
            for (int u = 0; u < depth_image.cols; ++u) {
                float depth = depth_image.at<float>(v, u);

                if (depth > 0.0f) {  // Valid depth
                    // Convert pixel to 3D point
                    float x = (u - intrinsics.cx) * depth / intrinsics.fx;
                    float y = (v - intrinsics.cy) * depth / intrinsics.fy;
                    float z = depth;

                    // Get color (if available)
                    cv::Vec3b color = color_image.at<cv::Vec3b>(v, u);

                    // Add point to cloud
                    pcl::PointXYZRGB point;
                    point.x = x;
                    point.y = y;
                    point.z = z;
                    point.r = color[2];  // OpenCV uses BGR, PCL uses RGB
                    point.g = color[1];
                    point.b = color[0];

                    pre_allocated_cloud_->points.push_back(point);
                }
            }
        }

        pre_allocated_cloud_->width = pre_allocated_cloud_->points.size();
        return pre_allocated_cloud_;
    }
};
```

## Sensor Fusion with Depth Cameras

### Combining with LiDAR Data

```python
import numpy as np
from scipy.spatial.transform import Rotation as R

class MultiSensorFusion:
    def __init__(self):
        # Transformation from depth camera to LiDAR frame
        self.T_depth_to_lidar = self.get_camera_lidar_transform()

    def get_camera_lidar_transform(self):
        """Get transformation matrix from camera to LiDAR frame"""
        # This should be calibrated for your specific robot
        # Example: camera is 10cm forward and 5cm above LiDAR
        t = np.array([0.1, 0.0, 0.05])  # translation
        r = R.from_euler('xyz', [0, 0, 0])  # rotation (no rotation in this example)

        transform = np.eye(4)
        transform[:3, :3] = r.as_matrix()
        transform[:3, 3] = t

        return transform

    def fuse_depth_lidar(self, depth_points, lidar_points):
        """Fuse depth camera and LiDAR point clouds"""

        # Transform depth points to LiDAR frame
        depth_points_homogeneous = np.column_stack([
            depth_points,
            np.ones(len(depth_points))
        ])

        transformed_depth_points = (self.T_depth_to_lidar @
                                   depth_points_homogeneous.T).T[:, :3]

        # Combine both point clouds
        combined_points = np.vstack([lidar_points, transformed_depth_points])

        return combined_points

    def colorize_lidar_points(self, lidar_points, color_image, depth_image):
        """Project LiDAR points onto color image to get colors"""

        # Camera intrinsic parameters
        fx, fy = 525.0, 525.0
        cx, cy = 319.5, 239.5

        colored_points = []

        for point in lidar_points:
            # Project 3D point to 2D image coordinates
            x, y, z = point
            u = int(fx * x / z + cx)
            v = int(fy * y / z + cy)

            # Check if projection is within image bounds
            if (0 <= u < color_image.shape[1] and
                0 <= v < color_image.shape[0]):

                # Get color from image
                color = color_image[v, u]

                # Verify depth consistency
                if abs(depth_image[v, v] - z) < 0.1:  # 10cm tolerance
                    colored_point = {
                        'position': point,
                        'color': color,
                        'intensity': np.mean(color)  # Could use for intensity
                    }
                    colored_points.append(colored_point)

        return colored_points
```

## Troubleshooting Common Issues

### Issue: Depth values are all zero or invalid
**Solution**: Check image encoding format - should be '32FC1' for depth images.

### Issue: Depth image appears black
**Solution**: Verify that depth values are in meters and within expected range.

### Issue: Point cloud is empty
**Solution**: Check point cloud cutoff parameters and ensure depth values are valid.

### Issue: Performance is poor with high-resolution cameras
**Solution**: Reduce update rate or use image subsampling.

## Best Practices

### 1. Parameter Selection

Choose realistic parameters based on actual hardware:

```xml
<!-- Example: Intel RealSense D435 -->
<sensor name="realsense_camera" type="depth">
  <camera>
    <horizontal_fov>1.047</horizontal_fov> <!-- 60 degrees -->
    <image>
      <width>640</width>
      <height>480</height>
      <format>R8G8B8</format>
    </image>
    <clip>
      <near>0.2</near> <!-- 20cm minimum -->
      <far>10.0</far>  <!-- 10m maximum -->
    </clip>
  </camera>
</sensor>
```

### 2. Memory Management

Efficiently handle large RGB-D data:

```cpp
// Use appropriate data types
// 32-bit float for depth (more accurate than 16-bit)
// Properly sized buffers for image processing

// Consider processing data in chunks for large images
void processLargeImage(const cv::Mat& large_image) {
    const int chunk_size = 100;  // Process in 100-row chunks

    for (int y = 0; y < large_image.rows; y += chunk_size) {
        int height = std::min(chunk_size, large_image.rows - y);
        cv::Mat chunk = large_image.rowRange(y, y + height);

        // Process chunk
        processImageChunk(chunk);
    }
}
```

## Summary

Depth camera simulation in Gazebo provides both color and depth information essential for advanced robotics applications. By understanding depth camera configuration, data processing techniques, and fusion with other sensors, you can create realistic RGB-D simulations that match real-world sensor characteristics.

## Next Steps

Now that you understand depth camera simulation, continue to [IMU Simulation](./imu-simulation.md) to learn about inertial measurement unit simulation and processing.