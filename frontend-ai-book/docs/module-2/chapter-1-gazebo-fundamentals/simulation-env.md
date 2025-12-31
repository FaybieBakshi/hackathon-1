---
sidebar_position: 3
title: "Simulation Environment Setup"
---

# Simulation Environment Setup

## Overview

This section covers how to set up and configure your Gazebo simulation environment. You'll learn to create realistic physics-enabled worlds with proper environmental parameters, lighting, and objects that interact according to real-world physics principles.

## World File Structure

Gazebo worlds are defined using SDF (Simulation Description Format) files. Here's the basic structure:

```xml
<?xml version="1.0" ?>
<sdf version="1.7">
  <world name="my_world">
    <!-- World properties -->
    <physics type="ode">
      <gravity>0 0 -9.8</gravity>
    </physics>

    <!-- Environment elements -->
    <light name="sun" type="directional">
      <pose>0 0 10 0 0 0</pose>
      <diffuse>0.8 0.8 0.8 1</diffuse>
      <specular>0.2 0.2 0.2 1</specular>
      <attenuation>
        <range>1000</range>
        <constant>0.9</constant>
        <linear>0.01</linear>
        <quadratic>0.001</quadratic>
      </attenuation>
      <direction>-0.5 0.1 -0.9</direction>
    </light>

    <!-- Ground plane -->
    <model name="ground_plane">
      <static>true</static>
      <link name="link">
        <collision name="collision">
          <geometry>
            <plane>
              <normal>0 0 1</normal>
              <size>100 100</size>
            </plane>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <plane>
              <normal>0 0 1</normal>
              <size>100 100</size>
            </plane>
          </geometry>
        </visual>
      </link>
    </model>

    <!-- Your robot or objects will go here -->
  </world>
</sdf>
```

## Creating a Basic Environment

### 1. Environment Configuration

Start by creating a basic world file that includes essential environmental parameters:

```xml
<sdf version="1.7">
  <world name="basic_physics_world">
    <!-- Physics engine configuration -->
    <physics type="ode">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1.0</real_time_factor>
      <real_time_update_rate>1000</real_time_update_rate>
      <gravity>0 0 -9.8</gravity>
    </physics>

    <!-- Environment lighting -->
    <light name="sun" type="directional">
      <pose>0 0 10 0 0 0</pose>
      <diffuse>0.8 0.8 0.8 1</diffuse>
      <specular>0.2 0.2 0.2 1</specular>
      <direction>-0.5 0.1 -0.9</direction>
    </light>

    <!-- Ground plane -->
    <model name="ground_plane">
      <static>true</static>
      <link name="link">
        <collision name="collision">
          <geometry>
            <plane>
              <normal>0 0 1</normal>
              <size>10 10</size>
            </plane>
          </geometry>
          <surface>
            <friction>
              <ode>
                <mu>1.0</mu>
                <mu2>1.0</mu2>
              </ode>
            </friction>
          </surface>
        </collision>
        <visual name="visual">
          <geometry>
            <plane>
              <normal>0 0 1</normal>
              <size>10 10</size>
            </plane>
          </geometry>
          <material>
            <ambient>0.7 0.7 0.7 1</ambient>
            <diffuse>0.7 0.7 0.7 1</diffuse>
          </material>
        </visual>
      </link>
    </model>
  </world>
</sdf>
```

### 2. Adding Obstacles and Interactive Objects

Add objects that will interact with your robot:

```xml
<!-- Cube obstacle -->
<model name="cube_obstacle">
  <pose>-2 0 0.5 0 0 0</pose>
  <link name="link">
    <collision name="collision">
      <geometry>
        <box>
          <size>1 1 1</size>
        </box>
      </geometry>
    </collision>
    <visual name="visual">
      <geometry>
        <box>
          <size>1 1 1</size>
        </box>
      </geometry>
      <material>
        <ambient>0.8 0.2 0.2 1</ambient>
        <diffuse>0.8 0.2 0.2 1</diffuse>
      </material>
    </visual>
    <inertial>
      <mass>1.0</mass>
      <inertia>
        <ixx>0.1667</ixx>
        <ixy>0</ixy>
        <ixz>0</ixz>
        <iyy>0.1667</iyy>
        <iyz>0</iyz>
        <izz>0.1667</izz>
      </inertia>
    </inertial>
  </link>
</model>

<!-- Sphere object -->
<model name="sphere_object">
  <pose>2 0 0.5 0 0 0</pose>
  <link name="link">
    <collision name="collision">
      <geometry>
        <sphere>
          <radius>0.5</radius>
        </sphere>
      </geometry>
    </collision>
    <visual name="visual">
      <geometry>
        <sphere>
          <radius>0.5</radius>
        </sphere>
      </geometry>
      <material>
        <ambient>0.2 0.2 0.8 1</ambient>
        <diffuse>0.2 0.2 0.8 1</diffuse>
      </material>
    </visual>
    <inertial>
      <mass>0.5</mass>
      <inertia>
        <ixx>0.05</ixx>
        <ixy>0</ixy>
        <ixz>0</ixz>
        <iyy>0.05</iyy>
        <iyz>0</iyz>
        <izz>0.05</izz>
      </inertia>
    </inertial>
  </link>
</model>
```

## Advanced Environment Configuration

### 1. Custom Physics Properties

Configure advanced physics parameters for specific simulation needs:

```xml
<physics type="ode">
  <max_step_size>0.001</max_step_size>
  <real_time_factor>1.0</real_time_factor>
  <real_time_update_rate>1000</real_time_update_rate>
  <gravity>0 0 -9.8</gravity>
  <ode>
    <solver>
      <type>quick</type>
      <iters>100</iters>
      <sor>1.3</sor>
    </solver>
    <constraints>
      <cfm>0.000001</cfm>
      <erp>0.2</erp>
      <contact_max_correcting_vel>100</contact_max_correcting_vel>
      <contact_surface_layer>0.001</contact_surface_layer>
    </constraints>
  </ode>
</physics>
```

### 2. Environmental Effects

Add environmental effects like wind, water, or atmospheric conditions:

```xml
<!-- Wind effects -->
<world name="windy_world">
  <physics type="ode">
    <gravity>0 0 -9.8</gravity>
  </physics>

  <!-- Wind volume -->
  <actor name="wind">
    <pose>0 0 2 0 0 0</pose>
    <animation>
      <filename>wind.dae</filename>
      <scale>1.0</scale>
      <interpolate_x>true</interpolate_x>
    </animation>
    <script>
      <loop>true</loop>
      <delay_start>0</delay_start>
      <trajectory id="0" type="circle">
        <waypoint>
          <time>0</time>
          <pose>0 0 2 0 0 0</pose>
        </waypoint>
        <waypoint>
          <time>20</time>
          <pose>2 0 2 0 0 0</pose>
        </waypoint>
        <waypoint>
          <time>40</time>
          <pose>0 0 2 0 0 0</pose>
        </waypoint>
      </trajectory>
    </script>
    <!-- Apply wind force to objects -->
    <wind>
      <linear_velocity>0.5 0 0</linear_velocity>
      <force>0.1 0 0</force>
    </wind>
  </actor>
</world>
```

## Environment Testing and Validation

### 1. Basic Testing

Test your environment by launching it:

```bash
# Launch the world file
gazebo /path/to/your/world_file.world

# Or use the command line
gazebo --verbose /path/to/your/world_file.world
```

### 2. Physics Validation

Validate that physics properties are working correctly:

```bash
# Check available topics
gz topic -l

# Monitor physics statistics
gz stats

# Check model states
gz topic -e -t /gazebo/model_states
```

### 3. Performance Monitoring

Monitor simulation performance:

```bash
# Check real-time factor
gz stats | grep "Real Time Factor"

# Monitor CPU usage
htop

# Check graphics performance
nvidia-smi  # For NVIDIA GPUs
```

## Environment Best Practices

### 1. Performance Optimization

- **Simplify collision geometry**: Use primitive shapes instead of complex meshes
- **Limit active objects**: Only enable physics for objects that need it
- **Adjust update rates**: Lower rates for less critical simulations
- **Use static models**: Mark immovable objects as static

### 2. Stability Considerations

- **Consistent time steps**: Use appropriate max_step_size for your simulation
- **Proper mass ratios**: Ensure masses are realistic and balanced
- **Adequate solver iterations**: Use enough iterations for stable contact

### 3. Realism vs. Performance

Balance realism with computational requirements:

```xml
<!-- For high-performance simulations -->
<physics type="ode">
  <max_step_size>0.01</max_step_size>  <!-- Larger time step -->
  <real_time_update_rate>100</real_time_update_rate>  <!-- Lower update rate -->
  <ode>
    <solver>
      <iters>20</iters>  <!-- Fewer iterations -->
    </solver>
  </ode>
</physics>

<!-- For high-accuracy simulations -->
<physics type="ode">
  <max_step_size>0.001</max_step_size>  <!-- Smaller time step -->
  <real_time_update_rate>1000</real_time_update_rate>  <!-- Higher update rate -->
  <ode>
    <solver>
      <iters>200</iters>  <!-- More iterations -->
    </solver>
  </ode>
</physics>
```

## Common Environment Configurations

### 1. Indoor Environment

For indoor robotics applications:

```xml
<world name="indoor_environment">
  <physics type="ode">
    <gravity>0 0 -9.8</gravity>
  </physics>

  <!-- Indoor lighting -->
  <light name="ceiling_light" type="point">
    <pose>0 0 3 0 0 0</pose>
    <diffuse>0.9 0.9 0.9 1</diffuse>
    <attenuation>
      <range>10</range>
      <constant>0.5</constant>
      <linear>0.1</linear>
      <quadratic>0.01</quadratic>
    </attenuation>
  </light>

  <!-- Walls -->
  <model name="wall_north">
    <pose>0 5 1.5 0 0 0</pose>
    <static>true</static>
    <link name="link">
      <collision name="collision">
        <geometry>
          <box>
            <size>10 0.2 3</size>
          </box>
        </geometry>
      </collision>
      <visual name="visual">
        <geometry>
          <box>
            <size>10 0.2 3</size>
          </box>
        </geometry>
      </visual>
    </link>
  </model>

  <!-- Add more walls as needed -->
</world>
```

### 2. Outdoor Environment

For outdoor robotics applications:

```xml
<world name="outdoor_environment">
  <physics type="ode">
    <gravity>0 0 -9.8</gravity>
  </physics>

  <!-- Natural lighting -->
  <light name="sun" type="directional">
    <pose>0 0 10 0 0 0</pose>
    <diffuse>1 1 1 1</diffuse>
    <specular>0.5 0.5 0.5 1</specular>
    <direction>-0.3 -0.3 -0.9</direction>
  </light>

  <!-- Terrain -->
  <model name="terrain">
    <static>true</static>
    <link name="link">
      <collision name="collision">
        <geometry>
          <heightmap>
            <uri>model://my_terrain/heightmap.png</uri>
            <size>100 100 10</size>
          </heightmap>
        </geometry>
      </collision>
      <visual name="visual">
        <geometry>
          <heightmap>
            <uri>model://my_terrain/heightmap.png</uri>
            <size>100 100 10</size>
          </heightmap>
        </geometry>
      </visual>
    </link>
  </model>
</world>
```

## Troubleshooting Common Issues

### Issue: Objects falling through the ground
**Solution**: Check collision properties and ensure proper surface parameters are set.

### Issue: Physics simulation is unstable
**Solution**: Reduce time step size or increase solver iterations.

### Issue: Performance is poor
**Solution**: Simplify collision geometry or reduce the number of active objects.

## Summary

Setting up a proper simulation environment is crucial for effective digital twin development. By understanding world file structure, physics parameters, and configuration best practices, you can create environments that accurately reflect real-world conditions for your robotics applications.

## Next Steps

Now that you understand how to set up simulation environments, proceed to [Practical Exercises](./practical-exercises.md) to apply your knowledge with hands-on activities.