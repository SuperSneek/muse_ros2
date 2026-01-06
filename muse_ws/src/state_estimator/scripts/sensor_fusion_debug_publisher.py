#!/usr/bin/env python3
"""
Sensor Fusion Debug Publisher
Publishes synthetic IMU, attitude, leg odometry, and base height messages
for Kalman filter debugging with configurable noise and state transitions.

IMU: ~1000 Hz
Other sensors: ~200 Hz
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

import numpy as np
import math
from enum import Enum

from sensor_msgs.msg import Imu
from state_estimator_msgs.msg import Attitude, LegOdometry, BaseHeight
from geometry_msgs.msg import Vector3, Quaternion


class PublishState(Enum):
    """Enum for different publishing states"""
    STATE_1 = 1
    TRANSITION = 2
    STATE_2 = 3


class SensorFusionDebugPublisher(Node):
    """
    Debug publisher for sensor fusion testing.
    
    Configuration parameters (ROS parameters):
    - state1_duration: Duration of state 1 (seconds, default=10.0)
    - transition_duration: Duration of transition from state 1 to state 2 (seconds, default=5.0)
    - loop_enabled: Whether to loop the sequence (default=false)
    
    State 1 (IMU):
    - imu_acc_mean: Mean acceleration [x, y, z] (default=[0.0, 0.0, 0.0])
    - imu_acc_noise_std: Std dev of acceleration noise (default=0.01)
    
    State 2 (IMU):
    - imu_acc_mean_s2: Mean acceleration in state 2 (default=[0.0, 0.0, 0.0])
    - imu_acc_noise_std_s2: Std dev of acceleration noise in state 2 (default=0.01)
    
    State 1 (Leg Odometry):
    - leg_odom_vel_mean: Mean velocity [x, y, z] (default=[0.0, 0.0, 0.0])
    - leg_odom_vel_noise_std: Std dev of velocity noise (default=0.005)
    
    State 2 (Leg Odometry):
    - leg_odom_vel_mean_s2: Mean velocity in state 2 (default=[0.0, 0.0, 0.0])
    - leg_odom_vel_noise_std_s2: Std dev of velocity noise in state 2 (default=0.005)
    
    Other:
    - initial_height: Initial height estimate (default=0.5)
    """
    
    def __init__(self):
        super().__init__('sensor_fusion_debug_publisher')
        
        # Declare parameters
        self.declare_parameter('state1_duration', 10.0)
        self.declare_parameter('transition_duration', 5.0)
        self.declare_parameter('loop_enabled', False)
        
        # IMU State 1
        self.declare_parameter('imu_acc_mean', [0.0, 0.0, 0.0])
        self.declare_parameter('imu_acc_noise_std', 0.01)
        
        # IMU State 2
        self.declare_parameter('imu_acc_mean_s2', [0.0, 0.0, 0.0])
        self.declare_parameter('imu_acc_noise_std_s2', 0.01)
        
        # Leg Odometry State 1
        self.declare_parameter('leg_odom_vel_mean', [0.0, 0.0, 0.0])
        self.declare_parameter('leg_odom_vel_noise_std', 0.005)
        
        # Leg Odometry State 2
        self.declare_parameter('leg_odom_vel_mean_s2', [0.0, 0.0, 0.0])
        self.declare_parameter('leg_odom_vel_noise_std_s2', 0.005)
        
        # Other
        self.declare_parameter('initial_height', 0.5)
        
        # Get parameters
        self.state1_duration = self.get_parameter('state1_duration').value
        self.transition_duration = self.get_parameter('transition_duration').value
        self.loop_enabled = self.get_parameter('loop_enabled').value
        
        self.imu_acc_mean_s1 = np.array(self.get_parameter('imu_acc_mean').value)
        self.imu_acc_noise_std_s1 = self.get_parameter('imu_acc_noise_std').value
        
        self.imu_acc_mean_s2 = np.array(self.get_parameter('imu_acc_mean_s2').value)
        self.imu_acc_noise_std_s2 = self.get_parameter('imu_acc_noise_std_s2').value
        
        self.leg_odom_vel_mean_s1 = np.array(self.get_parameter('leg_odom_vel_mean').value)
        self.leg_odom_vel_noise_std_s1 = self.get_parameter('leg_odom_vel_noise_std').value
        
        self.leg_odom_vel_mean_s2 = np.array(self.get_parameter('leg_odom_vel_mean_s2').value)
        self.leg_odom_vel_noise_std_s2 = self.get_parameter('leg_odom_vel_noise_std_s2').value
        
        self.initial_height = self.get_parameter('initial_height').value
        
        # QoS profile for sensor data
        sensor_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        # Publishers
        self.imu_pub = self.create_publisher(Imu, '/sensors/imu', sensor_qos)
        self.attitude_pub = self.create_publisher(Attitude, '/state_estimator/attitude', sensor_qos)
        self.leg_odom_pub = self.create_publisher(LegOdometry, '/sensors/leg_odometry', sensor_qos)
        self.base_height_pub = self.create_publisher(BaseHeight, '/state_estimator/base_height', sensor_qos)
        
        # State variables
        self.start_time = None
        self.current_state = PublishState.STATE_1
        self.height_published = False
        self.loop_count = 0
        
        # Publish rates
        self.imu_rate_hz = 1000
        self.other_rate_hz = 200
        
        # Create timers
        imu_period = 1.0 / self.imu_rate_hz
        other_period = 1.0 / self.other_rate_hz
        
        self.imu_timer = self.create_timer(imu_period, self.publish_imu)
        self.sensor_timer = self.create_timer(other_period, self.publish_other_sensors)
        
        # Attitude state (simulated)
        self.roll = 0.0
        self.pitch = 0.0
        self.yaw = 0.0
        self.angular_velocity = np.array([0.0, 0.0, 0.0])
        
        self.get_logger().info(
            f"Sensor Fusion Debug Publisher Started\n"
            f"  IMU Rate: {self.imu_rate_hz} Hz\n"
            f"  Other Sensors Rate: {self.other_rate_hz} Hz\n"
            f"  State 1 Duration: {self.state1_duration} s\n"
            f"  Transition Duration: {self.transition_duration} s\n"
            f"  Loop Enabled: {self.loop_enabled}\n"
            f"  State 1 IMU Mean: {self.imu_acc_mean_s1}\n"
            f"  State 1 IMU Noise Std: {self.imu_acc_noise_std_s1}\n"
            f"  State 2 IMU Mean: {self.imu_acc_mean_s2}\n"
            f"  State 2 IMU Noise Std: {self.imu_acc_noise_std_s2}\n"
            f"  State 1 Leg Odom Mean: {self.leg_odom_vel_mean_s1}\n"
            f"  State 1 Leg Odom Noise Std: {self.leg_odom_vel_noise_std_s1}\n"
            f"  State 2 Leg Odom Mean: {self.leg_odom_vel_mean_s2}\n"
            f"  State 2 Leg Odom Noise Std: {self.leg_odom_vel_noise_std_s2}"
        )
    
    def get_elapsed_time(self):
        """Get elapsed time since start"""
        if self.start_time is None:
            self.start_time = self.get_clock().now()
            return 0.0
        return (self.get_clock().now() - self.start_time).nanoseconds / 1e9
    
    def update_state(self, elapsed_time):
        """Update current state based on elapsed time"""
        state1_end = self.state1_duration
        transition_end = state1_end + self.transition_duration
        total_duration = transition_end
        
        if self.loop_enabled:
            elapsed_time = elapsed_time % total_duration
        
        if elapsed_time < state1_end:
            self.current_state = PublishState.STATE_1
            self.transition_progress = 0.0
        elif elapsed_time < transition_end:
            self.current_state = PublishState.TRANSITION
            self.transition_progress = (elapsed_time - state1_end) / self.transition_duration
        else:
            self.current_state = PublishState.STATE_2
            self.transition_progress = 1.0
    
    def interpolate_value(self, val1, val2, progress):
        """Linear interpolation between two values"""
        return val1 * (1.0 - progress) + val2 * progress
    
    def get_current_imu_params(self):
        """Get current IMU parameters based on state"""
        if self.current_state == PublishState.STATE_1:
            return self.imu_acc_mean_s1, self.imu_acc_noise_std_s1
        elif self.current_state == PublishState.STATE_2:
            return self.imu_acc_mean_s2, self.imu_acc_noise_std_s2
        else:  # TRANSITION
            mean = np.array([
                self.interpolate_value(self.imu_acc_mean_s1[i], self.imu_acc_mean_s2[i], self.transition_progress)
                for i in range(3)
            ])
            noise = self.interpolate_value(
                self.imu_acc_noise_std_s1,
                self.imu_acc_noise_std_s2,
                self.transition_progress
            )
            return mean, noise
    
    def get_current_leg_odom_params(self):
        """Get current leg odometry parameters based on state"""
        if self.current_state == PublishState.STATE_1:
            return self.leg_odom_vel_mean_s1, self.leg_odom_vel_noise_std_s1
        elif self.current_state == PublishState.STATE_2:
            return self.leg_odom_vel_mean_s2, self.leg_odom_vel_noise_std_s2
        else:  # TRANSITION
            mean = np.array([
                self.interpolate_value(self.leg_odom_vel_mean_s1[i], self.leg_odom_vel_mean_s2[i], self.transition_progress)
                for i in range(3)
            ])
            noise = self.interpolate_value(
                self.leg_odom_vel_noise_std_s1,
                self.leg_odom_vel_noise_std_s2,
                self.transition_progress
            )
            return mean, noise
    
    def publish_imu(self):
        """Publish IMU message at 1000 Hz"""
        elapsed_time = self.get_elapsed_time()
        self.update_state(elapsed_time)
        
        mean, noise = self.get_current_imu_params()
        
        # Generate noisy acceleration
        acc = mean + np.random.normal(0.0, noise, 3)
        
        msg = Imu()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'imu_link'
        
        msg.linear_acceleration.x = acc[0]
        msg.linear_acceleration.y = acc[1]
        msg.linear_acceleration.z = acc[2]
        
        # Angular velocity
        msg.angular_velocity.x = self.angular_velocity[0]
        msg.angular_velocity.y = self.angular_velocity[1]
        msg.angular_velocity.z = self.angular_velocity[2]
        
        # Covariance (diagonal)
        cov = noise ** 2
        msg.linear_acceleration_covariance = [
            cov, 0.0, 0.0,
            0.0, cov, 0.0,
            0.0, 0.0, cov
        ]
        
        self.imu_pub.publish(msg)
    
    def publish_other_sensors(self):
        """Publish attitude, leg odometry, and base height at 200 Hz"""
        elapsed_time = self.get_elapsed_time()
        self.update_state(elapsed_time)
        
        # Publish attitude
        self.publish_attitude(elapsed_time)
        
        # Publish leg odometry
        self.publish_leg_odometry()
        
        # Publish base height once
        if not self.height_published:
            self.publish_base_height()
            self.height_published = True
    
    def publish_attitude(self, elapsed_time):
        """Publish attitude message"""
        # Simulate slow changes in orientation
        self.roll = 0.05 * math.sin(elapsed_time * 0.2)
        self.pitch = 0.05 * math.cos(elapsed_time * 0.15)
        # yaw keeps increasing
        self.yaw = elapsed_time * 0.1
        
        # Convert RPY to quaternion
        quat = self.rpy_to_quaternion(self.roll, self.pitch, self.yaw)
        
        msg = Attitude()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'
        
        msg.quaternion = [quat[3], quat[0], quat[1], quat[2]]  # [w, x, y, z]
        msg.angular_velocity = list(self.angular_velocity)
        
        self.attitude_pub.publish(msg)
    
    def publish_leg_odometry(self):
        """Publish leg odometry message"""
        mean, noise = self.get_current_leg_odom_params()
        
        # Generate noisy velocity
        vel = mean + np.random.normal(0.0, noise, 3)
        
        msg = LegOdometry()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'
        
        msg.base_velocity = list(vel)
        
        self.leg_odom_pub.publish(msg)
    
    def publish_base_height(self):
        """Publish base height message (only once)"""
        msg = BaseHeight()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'world'
        msg.height = self.initial_height
        
        self.get_logger().info(f"Publishing initial height: {self.initial_height} m")
        self.base_height_pub.publish(msg)
    
    @staticmethod
    def rpy_to_quaternion(roll, pitch, yaw):
        """Convert roll, pitch, yaw to quaternion [x, y, z, w]"""
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        cp = math.cos(pitch * 0.5)
        sp = math.sin(pitch * 0.5)
        cr = math.cos(roll * 0.5)
        sr = math.sin(roll * 0.5)
        
        w = cr * cp * cy + sr * sp * sy
        x = sr * cp * cy - cr * sp * sy
        y = cr * sp * cy + sr * cp * sy
        z = cr * cp * sy - sr * sp * cy
        
        return [x, y, z, w]


def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = SensorFusionDebugPublisher()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
