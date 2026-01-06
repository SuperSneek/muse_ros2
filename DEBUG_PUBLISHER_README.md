# Sensor Fusion Debug Publisher

A ROS 2 Python script for publishing synthetic IMU, attitude, leg odometry, and base height messages to debug the Kalman filter in the sensor fusion plugin.

## Features

- **IMU Publishing**: 1000 Hz publication rate for realistic IMU sensor feedback
- **Other Sensors**: 200 Hz publication rate (attitude, leg odometry)
- **State Transitions**: Test filter behavior with two different states and smooth transitions
- **Configurable Noise**: Adjust noise levels for each sensor and state
- **Configurable Means**: Set different mean values to simulate different motion states
- **Smooth Transitions**: Linearly interpolate between states over a configurable duration

## Message Types Published

1. **IMU** (`sensor_msgs/msg/Imu`):
   - Linear acceleration with noise
   - Angular velocity
   - Covariance matrices

2. **Attitude** (`state_estimator_msgs/msg/Attitude`):
   - Quaternion orientation
   - Angular velocity
   - Simulated slow rotation

3. **Leg Odometry** (`state_estimator_msgs/msg/LegOdometry`):
   - Base velocity with noise
   - In body frame

4. **Base Height** (`state_estimator_msgs/msg/BaseHeight`):
   - Initial height estimate (published once)

## Configuration

Edit [sensor_fusion_debug_config.yaml](config/sensor_fusion_debug_config.yaml) to customize:

### Timing Parameters
- `state1_duration`: How long to stay in state 1 (seconds)
- `transition_duration`: How long to transition from state 1 to state 2 (seconds)
- `loop_enabled`: Whether to loop the sequence indefinitely

### State 1 Parameters
- `imu_acc_mean`: [x, y, z] acceleration mean in m/s²
- `imu_acc_noise_std`: Noise standard deviation for IMU
- `leg_odom_vel_mean`: [x, y, z] velocity mean in m/s
- `leg_odom_vel_noise_std`: Noise standard deviation for leg odometry

### State 2 Parameters
- Same as State 1 but with `_s2` suffix

### Other
- `initial_height`: Initial height estimate for the robot

## Usage

### Basic Launch (with default config)
```bash
cd ~/kamaro/muse
source muse_ws/install/setup.bash
ros2 launch state_estimator sensor_fusion_debug.launch.py
```

### With Custom Duration
```bash
ros2 launch state_estimator sensor_fusion_debug.launch.py \
    state1_duration:=15.0 \
    transition_duration:=10.0
```

### Run Script Directly
```bash
ros2 run state_estimator sensor_fusion_debug_publisher.py
```

### With Sensor Fusion Node
In one terminal:
```bash
cd ~/kamaro/muse/muse_ws
source install/setup.bash
ros2 launch state_estimator sensor_fusion.launch.py  # or your launch command
```

In another terminal:
```bash
ros2 launch state_estimator sensor_fusion_debug.launch.py
```

## Example Scenarios

### 1. Test at Rest with Noisy Measurements
```yaml
state1_duration: 30.0
transition_duration: 0.0
loop_enabled: false

imu_acc_mean: [0.0, 0.0, 0.0]
imu_acc_noise_std: 0.02
leg_odom_vel_mean: [0.0, 0.0, 0.0]
leg_odom_vel_noise_std: 0.01
```

### 2. Test Forward Acceleration
```yaml
state1_duration: 5.0
transition_duration: 3.0
loop_enabled: false

# At rest
imu_acc_mean: [0.0, 0.0, 0.0]
imu_acc_noise_std: 0.01
leg_odom_vel_mean: [0.0, 0.0, 0.0]
leg_odom_vel_noise_std: 0.005

# Moving forward
imu_acc_mean_s2: [1.0, 0.0, 0.0]
imu_acc_noise_std_s2: 0.015
leg_odom_vel_mean_s2: [0.5, 0.0, 0.0]
leg_odom_vel_noise_std_s2: 0.008
```

### 3. Test Side Motion
```yaml
state1_duration: 5.0
transition_duration: 3.0
loop_enabled: true

imu_acc_mean: [0.0, 0.0, 0.0]
imu_acc_noise_std: 0.01
leg_odom_vel_mean: [0.0, 0.0, 0.0]
leg_odom_vel_noise_std: 0.005

# Lateral motion
imu_acc_mean_s2: [0.0, 0.5, 0.0]
imu_acc_noise_std_s2: 0.02
leg_odom_vel_mean_s2: [0.0, 0.3, 0.0]
leg_odom_vel_noise_std_s2: 0.01
```

### 4. Test Vertical Motion
```yaml
state1_duration: 5.0
transition_duration: 2.0
loop_enabled: false

imu_acc_mean: [0.0, 0.0, 0.0]
imu_acc_noise_std: 0.01
leg_odom_vel_mean: [0.0, 0.0, 0.0]
leg_odom_vel_noise_std: 0.005

# Jumping/hopping (vertical acceleration + upward velocity)
imu_acc_mean_s2: [0.0, 0.0, 2.0]
imu_acc_noise_std_s2: 0.02
leg_odom_vel_mean_s2: [0.0, 0.0, 0.5]
leg_odom_vel_noise_std_s2: 0.01
```

## Debugging Tips

1. **Monitor Filter Output**: Watch the sensor fusion node's odometry output while running the debug publisher
2. **Check Convergence**: In state 1 (at rest), the velocity estimates should converge to near zero
3. **Test Transitions**: Use `transition_duration` to see how the filter responds to changes
4. **Tune Noise Levels**: If filter output is too noisy, increase `Q` and `R` matrices in the plugin config
5. **Check Logs**: The debug publisher logs state transitions:
   ```
   [INFO] Sensor Fusion Debug Publisher Started
   ...
   ```

## ROS 2 Topic Information

- IMU Topic: `/sensors/imu` (1000 Hz)
- Attitude Topic: `/state_estimator/attitude` (200 Hz)
- Leg Odometry Topic: `/sensors/leg_odometry` (200 Hz)
- Base Height Topic: `/state_estimator/base_height` (once at startup)
- Output Topic: `/sensors/odometry` (from sensor fusion filter)

Monitor topics with:
```bash
ros2 topic echo /sensors/imu
ros2 topic hz /sensors/imu
```

## Implementation Details

- The script uses ROS 2 parameters for full configurability
- Linear interpolation is used for smooth state transitions
- Gaussian noise is added to all sensor measurements
- Angular velocity is simulated as slowly rotating in roll/pitch/yaw
- Base height is published only once (latching behavior)
