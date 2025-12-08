# IMU Odometry Plugin

## Overview
The IMU Odometry Plugin estimates the robot's odometry using raw IMU measurements with manual filtering. It integrates acceleration to compute velocity and position, using the IMU's quaternion for orientation.

## Features
- Manual filtering with moving average and low-pass filters
- Gravity compensation from accelerometer data
- Configurable initial height estimate
- Velocity damping to reduce drift when stationary
- Z-axis stabilization to maintain height
- Publishes standard `nav_msgs/Odometry` messages

## Topics

### Subscribed Topics
- `/imu/data` (sensor_msgs/Imu): Raw IMU measurements including orientation, angular velocity, and linear acceleration

### Published Topics
- `/state_estimator/imu_odometry` (nav_msgs/Odometry): Estimated odometry

## Configuration Parameters

Edit `config/imu_odom.yaml` to configure the plugin:

```yaml
imu_odom_plugin:
  plugin: "ImuOdomPlugin"
  
  # Input topic
  imu_topic: "/imu/data"
  
  # Output topic
  output_topic: "/state_estimator/imu_odometry"
  
  # Frame IDs
  output_frame_id: "odom"
  child_frame_id: "base_link"
  
  # Initial conditions
  initial_height: 0.3  # Initial height estimate in meters
  
  # Filter settings
  velocity_filter_alpha: 0.1        # Low-pass filter coefficient (0-1)
  filter_window_size: 5             # Moving average window size
  position_drift_compensation: 0.01  # Velocity damping factor
  remove_gravity: true              # Remove gravity from measurements
```

### Key Parameters

- **initial_height**: The starting height of the robot above ground (meters). Adjust based on your robot's dimensions.
- **velocity_filter_alpha**: Low-pass filter coefficient for velocity (0-1). Lower values = smoother but slower response. Default: 0.1
- **filter_window_size**: Number of samples for moving average filter on acceleration and gyroscope. Default: 5
- **position_drift_compensation**: Damping factor applied to velocity when robot appears stationary (0-1). Higher values reduce drift but may affect motion tracking. Default: 0.01
- **remove_gravity**: When true, compensates for gravity in the acceleration measurements. Should be true for odometry. Default: true

## How It Works

1. **Orientation**: Uses the quaternion from the IMU directly (assumed to be filtered by the sensor)
2. **Angular Velocity**: Applies moving average filter to gyroscope data
3. **Acceleration Filtering**: 
   - Applies moving average filter to raw acceleration
   - Transforms to world frame using orientation
   - Removes gravity vector
   - Applies exponential smoothing (low-pass filter)
4. **Velocity Integration**: Integrates filtered acceleration with velocity damping when stationary
5. **Position Integration**: Integrates velocity with Z-axis stabilization
6. **Drift Mitigation**:
   - Velocity damping when acceleration is low
   - Z-axis pulled toward initial height when nearly stationary
   - Multiple cascaded filters reduce noise accumulation

## Usage

### Adding to Launch File

To use this plugin, add it to your state estimator configuration:

```python
imu_odom_config = os.path.join(
    get_package_share_directory('state_estimator'),
    'config',
    'imu_odom.yaml'
)

Node(
    package='state_estimator',
    executable='state_estimator_node',
    name='imu_odom_estimator',
    parameters=[imu_odom_config],
    output='screen'
)
```

### Testing

1. Build the package:
   ```bash
   cd ~/muse_ros2/muse_ws
   colcon build --symlink-install --packages-select state_estimator
   source install/setup.bash
   ```

2. Launch the state estimator with the plugin:
   ```bash
   ros2 launch state_estimator state_estimator.launch.py
   ```

3. Monitor the odometry output:
   ```bash
   ros2 topic echo /state_estimator/imu_odometry
   ```

4. Visualize in RViz:
   ```bash
   rviz2
   ```
   Add an Odometry display and set the topic to `/state_estimator/imu_odometry`.

## Limitations

- **Drift**: Even with filtered measurements, position estimates will drift over time. This is inherent to dead-reckoning from inertial measurements.
- **Z-axis constraint**: The plugin attempts to maintain the initial height when the robot is stationary (vertical velocity < 0.01 m/s) to prevent z-axis drift.
- **No loop closure**: This plugin does not perform any loop closure or global position correction. Consider fusing with other sensors (GPS, vision, leg odometry) for better accuracy.

## Recommendations

1. **Use filtered velocity**: Always set `use_filtered_velocity: true` to leverage the IMU's onboard filtering.
2. **Set accurate initial height**: Measure your robot's base link height above ground and set `initial_height` accordingly.
3. **Sensor fusion**: Combine this odometry with other sources (leg odometry, visual odometry, GPS) using a sensor fusion plugin or EKF for better results.
4. **Calibration**: Ensure your IMU is properly calibrated for best performance.

## Notes

- The velocity in the odometry message is expressed in the body frame (child_frame_id).
- Position is expressed in the world frame (output_frame_id).
- Covariance values are set to indicate uncertainty due to integration drift.
