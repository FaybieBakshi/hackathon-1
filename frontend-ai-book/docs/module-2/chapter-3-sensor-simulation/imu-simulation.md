---
sidebar_position: 12
title: "IMU Simulation"
---

# IMU Simulation in Gazebo

## Overview

This section covers Inertial Measurement Unit (IMU) sensor simulation in Gazebo. You'll learn to configure realistic IMU sensors, generate orientation and acceleration data, and work with IMU messages that match real-world sensor characteristics.

## IMU Fundamentals

### What is an IMU?

An Inertial Measurement Unit (IMU) is a sensor that measures:
- **Acceleration**: Linear acceleration in 3 axes (x, y, z)
- **Angular Velocity**: Rotational velocity in 3 axes (roll, pitch, yaw)
- **Orientation**: 3D orientation (often derived from other measurements)

### IMU Applications in Robotics

- **Attitude Estimation**: Determining robot orientation
- **Motion Tracking**: Monitoring robot movement and rotation
- **Stabilization**: Feedback for balance and control systems
- **SLAM**: Providing motion estimates for mapping and localization

### Types of IMU Sensors

1. **Accelerometers**: Measure linear acceleration
2. **Gyroscopes**: Measure angular velocity
3. **Magnetometers**: Measure magnetic field (for heading)
4. **6-axis IMUs**: Accelerometer + gyroscope
5. **9-axis IMUs**: Accelerometer + gyroscope + magnetometer

## Gazebo IMU Implementation

### IMU Sensor Plugin

Gazebo uses the `libgazebo_ros_imu_sensor.so` plugin to simulate IMU sensors:

```xml
<sensor name="imu_sensor" type="imu">
  <always_on>true</always_on>
  <update_rate>100</update_rate>
  <imu>
    <angular_velocity>
      <x>
        <noise type="gaussian">
          <mean>0.0</mean>
          <stddev>2e-4</stddev>
          <bias_mean>0.0000075</bias_mean>
          <bias_stddev>0.1</bias_stddev>
        </noise>
      </x>
      <y>
        <noise type="gaussian">
          <mean>0.0</mean>
          <stddev>2e-4</stddev>
          <bias_mean>0.0000075</bias_mean>
          <bias_stddev>0.1</bias_stddev>
        </noise>
      </y>
      <z>
        <noise type="gaussian">
          <mean>0.0</mean>
          <stddev>2e-4</stddev>
          <bias_mean>0.0000075</bias_mean>
          <bias_stddev>0.1</bias_stddev>
        </noise>
      </z>
    </angular_velocity>
    <linear_acceleration>
      <x>
        <noise type="gaussian">
          <mean>0.0</mean>
          <stddev>1.7e-2</stddev>
          <bias_mean>0.1</bias_mean>
          <bias_stddev>0.001</bias_stddev>
        </noise>
      </x>
      <y>
        <noise type="gaussian">
          <mean>0.0</mean>
          <stddev>1.7e-2</stddev>
          <bias_mean>0.1</bias_mean>
          <bias_stddev>0.001</bias_stddev>
        </noise>
      </y>
      <z>
        <noise type="gaussian">
          <mean>0.0</mean>
          <stddev>1.7e-2</stddev>
          <bias_mean>0.1</bias_mean>
          <bias_stddev>0.001</bias_stddev>
        </noise>
      </z>
    </linear_acceleration>
  </imu>

  <plugin name="imu_plugin" filename="libgazebo_ros_imu_sensor.so">
    <ros>
      <namespace>imu</namespace>
      <remapping>~/out:=data</remapping>
    </ros>
    <initial_orientation_as_reference>false</initial_orientation_as_reference>
    <frame_name>imu_link</frame_name>
  </plugin>
</sensor>
```

### Key IMU Parameters

#### Noise Parameters
- **Mean**: Average noise offset
- **Stddev**: Standard deviation of noise
- **Bias Mean**: Systematic offset in measurements
- **Bias Stddev**: Random walk of bias over time

#### Update Rate
- **Frequency**: How often sensor data is published (Hz)
- **Typical Values**: 100Hz for high-performance IMUs, 50Hz for basic ones

#### Reference Frame
- **Initial Orientation**: Whether to use initial orientation as reference
- **Frame Name**: TF frame for the IMU

## Configuring Different IMU Types

### Basic IMU Configuration

```xml
<sensor name="basic_imu" type="imu">
  <always_on>true</always_on>
  <update_rate>50</update_rate>

  <imu>
    <angular_velocity>
      <x>
        <noise type="gaussian">
          <mean>0.0</mean>
          <stddev>0.01</stddev>
        </noise>
      </x>
      <y>
        <noise type="gaussian">
          <mean>0.0</mean>
          <stddev>0.01</stddev>
        </noise>
      </y>
      <z>
        <noise type="gaussian">
          <mean>0.0</mean>
          <stddev>0.01</stddev>
        </noise>
      </z>
    </angular_velocity>
    <linear_acceleration>
      <x>
        <noise type="gaussian">
          <mean>0.0</mean>
          <stddev>0.05</stddev>
        </noise>
      </x>
      <y>
        <noise type="gaussian">
          <mean>0.0</mean>
          <stddev>0.05</stddev>
        </noise>
      </y>
      <z>
        <noise type="gaussian">
          <mean>0.0</mean>
          <stddev>0.05</stddev>
        </noise>
      </z>
    </linear_acceleration>
  </imu>

  <plugin name="basic_imu_plugin" filename="libgazebo_ros_imu_sensor.so">
    <ros>
      <namespace>imu</namespace>
      <remapping>~/out:=data</remapping>
    </ros>
    <frame_name>imu_link</frame_name>
  </plugin>
</sensor>
```

### High-Performance IMU Configuration

```xml
<sensor name="high_performance_imu" type="imu">
  <always_on>true</always_on>
  <update_rate>200</update_rate>

  <imu>
    <angular_velocity>
      <x>
        <noise type="gaussian">
          <mean>0.0</mean>
          <stddev>1e-5</stddev> <!-- Very low noise -->
          <bias_mean>0.0</bias_mean>
          <bias_stddev>1e-6</bias_stddev> <!-- Low bias drift -->
        </noise>
      </x>
      <y>
        <noise type="gaussian">
          <mean>0.0</mean>
          <stddev>1e-5</stddev>
          <bias_mean>0.0</bias_mean>
          <bias_stddev>1e-6</bias_stddev>
        </noise>
      </y>
      <z>
        <noise type="gaussian">
          <mean>0.0</mean>
          <stddev>1e-5</stddev>
          <bias_mean>0.0</bias_mean>
          <bias_stddev>1e-6</bias_stddev>
        </noise>
      </z>
    </angular_velocity>
    <linear_acceleration>
      <x>
        <noise type="gaussian">
          <mean>0.0</mean>
          <stddev>1e-4</stddev> <!-- Very low noise -->
          <bias_mean>0.0</bias_mean>
          <bias_stddev>1e-5</bias_stddev> <!-- Low bias drift -->
        </noise>
      </x>
      <y>
        <noise type="gaussian">
          <mean>0.0</mean>
          <stddev>1e-4</stddev>
          <bias_mean>0.0</bias_mean>
          <bias_stddev>1e-5</bias_stddev>
        </noise>
      </y>
      <z>
        <noise type="gaussian">
          <mean>0.0</mean>
          <stddev>1e-4</stddev>
          <bias_mean>0.0</bias_mean>
          <bias_stddev>1e-5</bias_stddev>
        </noise>
      </z>
    </linear_acceleration>
  </imu>

  <plugin name="high_performance_imu_plugin" filename="libgazebo_ros_imu_sensor.so">
    <ros>
      <namespace>imu</namespace>
      <remapping>~/out:=data</remapping>
    </ros>
    <frame_name>imu_link</frame_name>
  </plugin>
</sensor>
```

## IMU Data Processing

### Understanding IMU Messages

IMU messages contain three main components:

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Vector3, Quaternion
import numpy as np
from scipy.spatial.transform import Rotation as R

class IMUProcessor(Node):
    def __init__(self):
        super().__init__('imu_processor')

        # Subscribe to IMU data
        self.subscription = self.create_subscription(
            Imu,
            '/imu/data',
            self.imu_callback,
            10)

        # Publisher for processed data
        self.orientation_publisher = self.create_publisher(
            Quaternion,
            '/imu/orientation',
            10)

        self.angular_velocity_publisher = self.create_publisher(
            Vector3,
            '/imu/angular_velocity',
            10)

        self.linear_acceleration_publisher = self.create_publisher(
            Vector3,
            '/imu/linear_acceleration',
            10)

    def imu_callback(self, msg):
        # Extract components from IMU message
        orientation = msg.orientation  # Quaternion (x, y, z, w)
        angular_velocity = msg.angular_velocity  # Vector3 (x, y, z)
        linear_acceleration = msg.linear_acceleration  # Vector3 (x, y, z)

        # Process orientation
        orientation_processed = self.process_orientation(orientation)

        # Process angular velocity
        angular_velocity_processed = self.process_angular_velocity(angular_velocity)

        # Process linear acceleration
        linear_acceleration_processed = self.process_linear_acceleration(
            linear_acceleration, orientation)

        # Publish processed data
        self.orientation_publisher.publish(orientation_processed)
        self.angular_velocity_publisher.publish(angular_velocity_processed)
        self.linear_acceleration_publisher.publish(linear_acceleration_processed)

    def process_orientation(self, orientation):
        """Process orientation quaternion"""
        # Normalize quaternion to ensure it's a valid rotation
        quat_array = np.array([orientation.x, orientation.y, orientation.z, orientation.w])
        normalized_quat = quat_array / np.linalg.norm(quat_array)

        # Convert back to message format
        processed_quat = Quaternion()
        processed_quat.x = normalized_quat[0]
        processed_quat.y = normalized_quat[1]
        processed_quat.z = normalized_quat[2]
        processed_quat.w = normalized_quat[3]

        return processed_quat

    def process_angular_velocity(self, angular_velocity):
        """Process angular velocity vector"""
        # Apply any necessary filtering or calibration
        processed_av = Vector3()
        processed_av.x = angular_velocity.x
        processed_av.y = angular_velocity.y
        processed_av.z = angular_velocity.z

        return processed_av

    def process_linear_acceleration(self, linear_acceleration, orientation):
        """Process linear acceleration considering gravity"""
        # Convert to numpy array
        accel_array = np.array([linear_acceleration.x,
                               linear_acceleration.y,
                               linear_acceleration.z])

        # Create rotation object from orientation
        quat = [orientation.x, orientation.y, orientation.z, orientation.w]
        rotation = R.from_quat(quat)

        # Transform acceleration to inertial frame to remove gravity
        # (assuming the IMU frame is oriented properly)
        gravity = np.array([0, 0, 9.81])  # Gravity vector in world frame
        gravity_in_imu_frame = rotation.inv().apply(gravity)

        # Remove gravity from measured acceleration
        linear_acceleration_no_gravity = accel_array - gravity_in_imu_frame

        # Convert back to message format
        processed_accel = Vector3()
        processed_accel.x = linear_acceleration_no_gravity[0]
        processed_accel.y = linear_acceleration_no_gravity[1]
        processed_accel.z = linear_acceleration_no_gravity[2]

        return processed_accel
```

### Orientation Estimation

```python
import numpy as np
from scipy.spatial.transform import Rotation as R

class OrientationEstimator:
    def __init__(self, initial_orientation=None):
        if initial_orientation is None:
            self.orientation = R.identity()
        else:
            # Initialize with given quaternion [x, y, z, w]
            self.orientation = R.from_quat(initial_orientation)

        self.angular_velocity_bias = np.zeros(3)
        self.last_timestamp = None

    def update_orientation(self, angular_velocity, linear_acceleration, timestamp):
        """Update orientation using gyroscope and accelerometer fusion"""

        if self.last_timestamp is None:
            self.last_timestamp = timestamp
            return self.orientation.as_quat()

        # Calculate time difference
        dt = timestamp - self.last_timestamp
        self.last_timestamp = timestamp

        # Remove bias from angular velocity
        corrected_angular_velocity = angular_velocity - self.angular_velocity_bias

        # Integrate angular velocity to get orientation change
        # Use small angle approximation: rotation_vector = angular_velocity * dt
        rotation_vector = corrected_angular_velocity * dt

        # Create incremental rotation
        incremental_rotation = R.from_rotvec(rotation_vector)

        # Update orientation
        self.orientation = incremental_rotation * self.orientation

        # Optionally correct with accelerometer data
        self.orientation = self.correct_with_accelerometer(
            self.orientation, linear_acceleration)

        return self.orientation.as_quat()

    def correct_with_accelerometer(self, current_orientation, linear_acceleration):
        """Correct orientation drift using accelerometer data"""
        # Normalize accelerometer reading
        accel_norm = linear_acceleration / np.linalg.norm(linear_acceleration)

        # Expected gravity direction in world frame
        expected_gravity = np.array([0, 0, -1])  # Assuming z-down coordinate system

        # Calculate correction rotation
        # This is a simplified approach - more sophisticated methods exist
        if np.dot(accel_norm, expected_gravity) > 0.5:  # Reasonable alignment
            # Calculate rotation needed to align measured gravity with expected
            rotation_axis = np.cross(expected_gravity, accel_norm)
            rotation_angle = np.arccos(np.clip(np.dot(expected_gravity, accel_norm), -1, 1))

            correction_rotation = R.from_rotvec(rotation_axis * rotation_angle * 0.1)  # 10% correction
            return correction_rotation * current_orientation

        return current_orientation

    def get_euler_angles(self):
        """Get orientation as Euler angles (roll, pitch, yaw)"""
        return self.orientation.as_euler('xyz', degrees=True)
```

### Kalman Filter for IMU Fusion

```python
import numpy as np

class IMUKalmanFilter:
    def __init__(self):
        # State: [orientation_x, orientation_y, orientation_z, orientation_w,
        #         angular_velocity_x, angular_velocity_y, angular_velocity_z]
        self.state_dim = 7
        self.state = np.zeros(self.state_dim)
        self.state[3] = 1.0  # Initialize with w=1 (identity quaternion)

        # Covariance matrix
        self.P = np.eye(self.state_dim) * 0.1

        # Process noise
        self.Q = np.eye(self.state_dim) * 0.01

        # Measurement noise
        self.R = np.eye(6) * 0.1  # 3 for angular velocity, 3 for acceleration

    def predict(self, angular_velocity, dt):
        """Prediction step using gyroscope data"""
        # Extract angular velocity from state
        omega = angular_velocity

        # State transition matrix for quaternion integration
        # For quaternion integration: q_dot = 0.5 * Omega * q
        # where Omega is the skew-symmetric matrix
        omega_skew = np.array([
            [0, -omega[0], -omega[1], -omega[2]],
            [omega[0], 0, omega[2], -omega[1]],
            [omega[1], -omega[2], 0, omega[0]],
            [omega[2], omega[1], -omega[0], 0]
        ])

        # Update quaternion state
        quat = self.state[:4]
        quat_dot = 0.5 * omega_skew @ quat
        self.state[:4] += quat_dot * dt

        # Normalize quaternion to maintain unit length
        self.state[:4] /= np.linalg.norm(self.state[:4])

        # Update angular velocity state (assuming constant velocity model)
        self.state[4:7] = angular_velocity

        # Predict covariance
        # Simplified Jacobian calculation
        F = np.eye(self.state_dim)
        # Add process noise
        self.P = F @ self.P @ F.T + self.Q

    def update(self, measured_angular_velocity, measured_acceleration):
        """Update step using measured data"""
        # Measurement model: we measure angular velocity and acceleration
        # Expected measurement from current state
        expected_angular_velocity = self.state[4:7]
        # Acceleration measurement is used differently in more complex implementations

        # Innovation (measurement residual)
        innovation = np.concatenate([
            measured_angular_velocity - expected_angular_velocity,
            measured_acceleration  # For more complex fusion
        ])

        # Innovation covariance
        H = np.zeros((6, self.state_dim))
        H[:3, 4:7] = np.eye(3)  # Angular velocity measurements map to state

        S = H @ self.P @ H.T + self.R

        # Kalman gain
        K = self.P @ H.T @ np.linalg.inv(S)

        # Update state
        self.state += K @ innovation

        # Normalize quaternion part
        self.state[:4] /= np.linalg.norm(self.state[:4])

        # Update covariance
        I = np.eye(self.state_dim)
        self.P = (I - K @ H) @ self.P
```

## Integration with Robot State

### TF Broadcasting with IMU Data

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
import tf_transformations

class IMUTFBroadcaster(Node):
    def __init__(self):
        super().__init__('imu_tf_broadcaster')

        # Create transform broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)

        # Subscribe to IMU data
        self.subscription = self.create_subscription(
            Imu,
            '/imu/data',
            self.imu_callback,
            10)

        self.robot_base_frame = 'base_link'
        self.imu_frame = 'imu_link'

    def imu_callback(self, msg):
        # Create transform from IMU data
        t = TransformStamped()

        # Set header
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = self.robot_base_frame
        t.child_frame_id = self.imu_frame

        # Set transform (for IMU, usually just the mounting position)
        # In this case, we're just broadcasting the orientation from IMU
        t.transform.translation.x = 0.0  # IMU position relative to base
        t.transform.translation.y = 0.0
        t.transform.translation.z = 0.1  # 10cm above base

        # Use orientation from IMU
        t.transform.rotation = msg.orientation

        # Broadcast transform
        self.tf_broadcaster.sendTransform(t)

    def get_orientation_from_imu(self, msg):
        """Extract orientation from IMU message with possible corrections"""
        # In real applications, you might want to integrate angular velocity
        # or use more sophisticated orientation estimation
        return msg.orientation
```

## Performance Considerations

### Efficient IMU Processing

```cpp
#include <sensor_msgs/msg/imu.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <geometry_msgs/msg/quaternion.hpp>

class EfficientIMUProcessor
{
private:
    rclcpp::Time last_update_time_;
    std::array<double, 3> bias_estimate_;
    std::array<double, 4> current_orientation_; // x, y, z, w
    bool initialized_;

public:
    EfficientIMUProcessor() : initialized_(false) {
        // Initialize bias estimate to zero
        bias_estimate_.fill(0.0);
        // Initialize orientation to identity quaternion
        current_orientation_[0] = current_orientation_[1] = current_orientation_[2] = 0.0;
        current_orientation_[3] = 1.0;
    }

    void processIMU(const sensor_msgs::msg::Imu::SharedPtr msg)
    {
        if (!initialized_) {
            initializeFromIMU(msg);
            initialized_ = true;
            last_update_time_ = msg->header.stamp;
            return;
        }

        // Calculate time difference efficiently
        rclcpp::Duration dt = msg->header.stamp - last_update_time_;
        double dt_sec = dt.seconds();

        if (dt_sec <= 0) return; // Invalid time

        // Update orientation using angular velocity
        updateOrientation(msg->angular_velocity, dt_sec);

        // Update bias estimate (simple moving average approach)
        updateBiasEstimate(msg->angular_velocity);

        last_update_time_ = msg->header.stamp;
    }

private:
    void initializeFromIMU(const sensor_msgs::msg::Imu::SharedPtr msg)
    {
        // Initialize orientation from accelerometer data if available
        // This is a simplified approach - real initialization is more complex
        std::array<double, 3> accel = {msg->linear_acceleration.x,
                                      msg->linear_acceleration.y,
                                      msg->linear_acceleration.z};

        // Calculate roll and pitch from accelerometer
        double roll = atan2(accel[1], accel[2]);
        double pitch = atan2(-accel[0], sqrt(accel[1]*accel[1] + accel[2]*accel[2]));

        // Convert to quaternion (yaw remains 0 initially)
        double cy = cos(yaw * 0.5);
        double sy = sin(yaw * 0.5);
        double cp = cos(pitch * 0.5);
        double sp = sin(pitch * 0.5);
        double cr = cos(roll * 0.5);
        double sr = sin(roll * 0.5);

        current_orientation_[0] = sr * cp * cy - cr * sp * sy;
        current_orientation_[1] = cr * sp * cy + sr * cp * sy;
        current_orientation_[2] = cr * cp * sy - sr * sp * cy;
        current_orientation_[3] = cr * cp * cy + sr * sp * sy;
    }

    void updateOrientation(const geometry_msgs::msg::Vector3& angular_vel, double dt)
    {
        // Apply bias correction
        std::array<double, 3> corrected_omega = {
            angular_vel.x - bias_estimate_[0],
            angular_vel.y - bias_estimate_[1],
            angular_vel.z - bias_estimate_[2]
        };

        // Integrate angular velocity to update orientation
        // Using first-order integration (better methods exist)
        std::array<double, 4> omega_quat = {0.0, corrected_omega[0], corrected_omega[1], corrected_omega[2]};

        // Calculate quaternion derivative: q_dot = 0.5 * omega_quat * q
        std::array<double, 4> quat_derivative;
        quat_derivative[0] = -0.5 * (omega_quat[1]*current_orientation_[1] +
                                   omega_quat[2]*current_orientation_[2] +
                                   omega_quat[3]*current_orientation_[3]);
        quat_derivative[1] = 0.5 * (omega_quat[0]*current_orientation_[1] +
                                  omega_quat[2]*current_orientation_[3] -
                                  omega_quat[3]*current_orientation_[2]);
        quat_derivative[2] = 0.5 * (omega_quat[0]*current_orientation_[2] -
                                  omega_quat[1]*current_orientation_[3] +
                                  omega_quat[3]*current_orientation_[1]);
        quat_derivative[3] = 0.5 * (omega_quat[0]*current_orientation_[3] +
                                  omega_quat[1]*current_orientation_[2] -
                                  omega_quat[2]*current_orientation_[1]);

        // Update orientation
        for (int i = 0; i < 4; ++i) {
            current_orientation_[i] += quat_derivative[i] * dt;
        }

        // Normalize quaternion
        double norm = sqrt(current_orientation_[0]*current_orientation_[0] +
                          current_orientation_[1]*current_orientation_[1] +
                          current_orientation_[2]*current_orientation_[2] +
                          current_orientation_[3]*current_orientation_[3]);
        if (norm > 0) {
            for (int i = 0; i < 4; ++i) {
                current_orientation_[i] /= norm;
            }
        }
    }

    void updateBiasEstimate(const geometry_msgs::msg::Vector3& angular_vel)
    {
        // Simple bias estimation - in practice, this would be more sophisticated
        const double bias_learning_rate = 0.001;

        bias_estimate_[0] += bias_learning_rate * (angular_vel.x - bias_estimate_[0]);
        bias_estimate_[1] += bias_learning_rate * (angular_vel.y - bias_estimate_[1]);
        bias_estimate_[2] += bias_learning_rate * (angular_vel.z - bias_estimate_[2]);
    }
};
```

## Sensor Fusion with IMU

### Combining IMU with Other Sensors

```python
import numpy as np
from scipy.spatial.transform import Rotation as R

class MultiSensorFusion:
    def __init__(self):
        # Initialize with IMU-only state
        self.orientation = R.identity()
        self.position = np.zeros(3)
        self.velocity = np.zeros(3)

        # Covariance matrix for extended Kalman filter
        self.P = np.eye(9) * 0.1  # Position, velocity, orientation

    def fuse_imu_odometry(self, imu_data, odometry_data, dt):
        """Fuse IMU and wheel odometry data"""

        # Predict state using IMU data
        predicted_state = self.predict_with_imu(imu_data, dt)

        # Update with odometry data
        corrected_state = self.update_with_odometry(predicted_state, odometry_data)

        return corrected_state

    def predict_with_imu(self, imu_data, dt):
        """Predict state using IMU measurements"""

        # Extract measurements
        angular_velocity = np.array([
            imu_data.angular_velocity.x,
            imu_data.angular_velocity.y,
            imu_data.angular_velocity.z
        ])

        linear_acceleration = np.array([
            imu_data.linear_acceleration.x,
            imu_data.linear_acceleration.y,
            imu_data.linear_acceleration.z
        ])

        # Update orientation
        rotation_vector = angular_velocity * dt
        incremental_rotation = R.from_rotvec(rotation_vector)
        self.orientation = incremental_rotation * self.orientation

        # Transform acceleration to world frame and integrate
        world_acceleration = self.orientation.apply(linear_acceleration)

        # Remove gravity (assuming we know the gravity direction)
        gravity = np.array([0, 0, 9.81])
        linear_acceleration_world = world_acceleration - gravity

        # Update velocity and position
        self.velocity += linear_acceleration_world * dt
        self.position += self.velocity * dt + 0.5 * linear_acceleration_world * dt**2

        return {
            'position': self.position.copy(),
            'velocity': self.velocity.copy(),
            'orientation': self.orientation.as_quat()
        }

    def update_with_odometry(self, predicted_state, odometry_data):
        """Update state with odometry measurements"""

        # Get odometry position and orientation
        odom_position = np.array([
            odometry_data.pose.pose.position.x,
            odometry_data.pose.pose.position.y,
            odometry_data.pose.pose.position.z
        ])

        odom_orientation = R.from_quat([
            odometry_data.pose.pose.orientation.x,
            odometry_data.pose.pose.orientation.y,
            odometry_data.pose.pose.orientation.z,
            odometry_data.pose.pose.orientation.w
        ])

        # Weighted fusion based on sensor covariances
        # This is simplified - real implementation would use proper Kalman filtering
        position_weight = 0.1  # Trust odometry for position
        orientation_weight = 0.3  # Balance IMU and odometry for orientation

        # Update position (trust odometry more)
        self.position = (1 - position_weight) * predicted_state['position'] + \
                       position_weight * odom_position

        # Update orientation (balance IMU and odometry)
        predicted_rot = R.from_quat(predicted_state['orientation'])
        fused_rotation = R.from_rotvec(
            (1 - orientation_weight) * predicted_rot.as_rotvec() +
            orientation_weight * odom_orientation.as_rotvec()
        )
        self.orientation = fused_rotation

        return {
            'position': self.position,
            'velocity': self.velocity,
            'orientation': self.orientation.as_quat()
        }
```

## Troubleshooting Common Issues

### Issue: IMU orientation is drifting
**Solution**: Implement proper bias estimation and correction algorithms.

### Issue: IMU readings are noisy
**Solution**: Verify noise parameters in the configuration match the target sensor.

### Issue: Gravity is not properly accounted for
**Solution**: Ensure coordinate frames are properly aligned and gravity compensation is implemented.

### Issue: IMU data is not being published
**Solution**: Check plugin configuration and frame names in the URDF.

## Best Practices

### 1. Realistic Noise Modeling

Configure noise parameters based on actual IMU specifications:

```xml
<!-- Example: ADIS16448 IMU realistic parameters -->
<sensor name="realistic_imu" type="imu">
  <imu>
    <angular_velocity>
      <x>
        <noise type="gaussian">
          <mean>0.0</mean>
          <stddev>0.00873</stddev> <!-- 0.5 deg/s in rad/s -->
          <bias_mean>0.0</bias_mean>
          <bias_stddev>0.0001745</bias_stddev> <!-- 0.01 deg/s in rad/s -->
        </noise>
      </x>
    </angular_velocity>
    <linear_acceleration>
      <x>
        <noise type="gaussian">
          <mean>0.0</mean>
          <stddev>0.00196</stddev> <!-- 200 μg in m/s² -->
          <bias_mean>0.0</bias_mean>
          <bias_stddev>0.0098</bias_stddev> <!-- 1 mg in m/s² -->
        </noise>
      </x>
    </linear_acceleration>
  </imu>
</sensor>
```

### 2. Proper Coordinate Frame Alignment

Ensure IMU frame aligns with robot coordinate system:

```xml
<!-- Mount IMU with proper orientation -->
<sensor name="imu_sensor" type="imu">
  <pose>0 0 0.1 0 0 0</pose> <!-- 10cm above base, no rotation -->
  <!-- IMU frame aligned with robot base frame -->
</sensor>
```

## Summary

IMU simulation in Gazebo provides crucial inertial data for robotics applications. By understanding IMU configuration, data processing techniques, and fusion with other sensors, you can create realistic inertial measurements that match real-world sensor characteristics. Proper noise modeling and bias estimation are essential for realistic simulation.

## Next Steps

Now that you understand IMU simulation, continue to [Sensor Fusion](./sensor-fusion.md) to learn how to combine multiple sensor data streams for enhanced perception capabilities.