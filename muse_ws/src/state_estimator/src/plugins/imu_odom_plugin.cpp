// Core
#include "state_estimator/plugin.hpp"
#include <rclcpp/rclcpp.hpp>

// ROS 2 messages
#include <sensor_msgs/msg/imu.hpp>
#include <nav_msgs/msg/odometry.hpp>

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <deque>

namespace state_estimator_plugins
{

class ImuOdomPlugin : public PluginBase
{
public:
    ImuOdomPlugin() = default;
    ~ImuOdomPlugin() override = default;

    std::string getName() override { return std::string("ImuOdom"); }
    std::string getDescription() override { return std::string("IMU-based Odometry Plugin"); }

    void initialize_() override {
        auto node = this->node_;
        if (!node) {
            throw std::runtime_error("ImuOdomPlugin: node_ is null");
        }

        RCLCPP_INFO(node->get_logger(), "Initializing ImuOdomPlugin...");

        // Initialize state
        position_.setZero();
        velocity_.setZero();
        velocity_filtered_.setZero();
        acceleration_filtered_.setZero();
        orientation_.setIdentity();
        angular_velocity_.setZero();
        initialized_ = false;
        last_time_ = rclcpp::Time(0, 0, RCL_ROS_TIME);

        // Load parameters
        loadParameters();

        // Set initial height
        position_.z() = initial_height_;
        RCLCPP_INFO(node->get_logger(), "Initial height set to: %.3f m", initial_height_);

        // Setup subscribers and publishers
        setupTopics();

        RCLCPP_INFO(node->get_logger(), "ImuOdomPlugin initialized successfully");
    }

    void shutdown_() override {}
    void pause_() override {}
    void resume_() override {}
    void reset_() override {
        position_.setZero();
        velocity_.setZero();
        velocity_filtered_.setZero();
        acceleration_filtered_.setZero();
        orientation_.setIdentity();
        angular_velocity_.setZero();
        position_.z() = initial_height_;
        initialized_ = false;
        last_time_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
        accel_buffer_.clear();
        gyro_buffer_.clear();
        RCLCPP_INFO(this->node_->get_logger(), "ImuOdomPlugin reset");
    }

private:
    // Subscribers and publishers
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;

    // State variables
    Eigen::Vector3d position_;
    Eigen::Vector3d velocity_;
    Eigen::Vector3d velocity_filtered_;
    Eigen::Vector3d acceleration_filtered_;
    Eigen::Quaterniond orientation_;
    Eigen::Vector3d angular_velocity_;
    rclcpp::Time last_time_;
    bool initialized_;

    // Filtering buffers for moving average
    std::deque<Eigen::Vector3d> accel_buffer_;
    std::deque<Eigen::Vector3d> gyro_buffer_;
    size_t filter_window_size_;

    // Gravity vector for compensation
    Eigen::Vector3d gravity_;

    // Parameters
    double initial_height_;
    std::string imu_topic_;
    std::string output_topic_;
    std::string output_frame_id_;
    std::string child_frame_id_;
    double velocity_filter_alpha_;
    double position_drift_compensation_;
    bool remove_gravity_;

    void loadParameters() {
        auto node = this->node_;

        // Topic names
        imu_topic_ = node->declare_parameter<std::string>(
            "imu_odom_plugin.imu_topic", "/imu/data");
        output_topic_ = node->declare_parameter<std::string>(
            "imu_odom_plugin.output_topic", "/state_estimator/imu_odometry");

        // Frame IDs
        output_frame_id_ = node->declare_parameter<std::string>(
            "imu_odom_plugin.output_frame_id", "odom");
        child_frame_id_ = node->declare_parameter<std::string>(
            "imu_odom_plugin.child_frame_id", "base_link");

        // Initial conditions
        initial_height_ = node->declare_parameter<double>(
            "imu_odom_plugin.initial_height", 0.06);

        // Filter parameters
        velocity_filter_alpha_ = node->declare_parameter<double>(
            "imu_odom_plugin.velocity_filter_alpha", 0.1);
        filter_window_size_ = node->declare_parameter<int>(
            "imu_odom_plugin.filter_window_size", 5);
        position_drift_compensation_ = node->declare_parameter<double>(
            "imu_odom_plugin.position_drift_compensation", 0.01);
        remove_gravity_ = node->declare_parameter<bool>(
            "imu_odom_plugin.remove_gravity", true);

        // Gravity vector (in world frame, z-up)
        gravity_ << 0.0, 0.0, 9.81;

        RCLCPP_INFO(node->get_logger(), "ImuOdomPlugin parameters loaded:");
        RCLCPP_INFO(node->get_logger(), "  imu_topic: %s", imu_topic_.c_str());
        RCLCPP_INFO(node->get_logger(), "  initial_height: %.3f m", initial_height_);
        RCLCPP_INFO(node->get_logger(), "  velocity_filter_alpha: %.3f", velocity_filter_alpha_);
        RCLCPP_INFO(node->get_logger(), "  filter_window_size: %zu", filter_window_size_);
        RCLCPP_INFO(node->get_logger(), "  remove_gravity: %s", remove_gravity_ ? "true" : "false");
    }

    void setupTopics() {
        auto node = this->node_;
        auto sensor_qos = rclcpp::SensorDataQoS();

        // Setup IMU subscriber
        imu_sub_ = node->create_subscription<sensor_msgs::msg::Imu>(
            imu_topic_, sensor_qos,
            std::bind(&ImuOdomPlugin::imuCallback, this, std::placeholders::_1));

        // Setup publisher
        odom_pub_ = node->create_publisher<nav_msgs::msg::Odometry>(
            output_topic_, rclcpp::QoS(10));

        RCLCPP_INFO(node->get_logger(), "Topics configured successfully");
    }

    Eigen::Vector3d applyMovingAverageFilter(std::deque<Eigen::Vector3d>& buffer, 
                                             const Eigen::Vector3d& new_value) {
        buffer.push_back(new_value);
        if (buffer.size() > filter_window_size_) {
            buffer.pop_front();
        }

        Eigen::Vector3d average = Eigen::Vector3d::Zero();
        for (const auto& val : buffer) {
            average += val;
        }
        return average / static_cast<double>(buffer.size());
    }

    void imuCallback(const sensor_msgs::msg::Imu::ConstSharedPtr msg) {
        auto current_time = rclcpp::Time(msg->header.stamp);

        // Extract orientation from IMU (already filtered by the sensor)
        orientation_.w() = msg->orientation.w;
        orientation_.x() = msg->orientation.x;
        orientation_.y() = msg->orientation.y;
        orientation_.z() = msg->orientation.z;
        orientation_.normalize();

        // Extract raw angular velocity
        Eigen::Vector3d raw_gyro(msg->angular_velocity.x, 
                                 msg->angular_velocity.y, 
                                 msg->angular_velocity.z);
        
        // Extract raw linear acceleration
        Eigen::Vector3d raw_accel(msg->linear_acceleration.x,
                                  msg->linear_acceleration.y,
                                  msg->linear_acceleration.z);

        // Apply moving average filter
        angular_velocity_ = applyMovingAverageFilter(gyro_buffer_, raw_gyro);
        Eigen::Vector3d accel_filtered = applyMovingAverageFilter(accel_buffer_, raw_accel);

        if (!initialized_) {
            // First callback - initialize
            last_time_ = current_time;
            velocity_.setZero();
            velocity_filtered_.setZero();
            acceleration_filtered_ = accel_filtered;
            
            initialized_ = true;
            RCLCPP_INFO(this->node_->get_logger(), 
                       "ImuOdomPlugin initialized at time %.3f", current_time.seconds());
            return;
        }

        // Compute time delta
        double dt = (current_time - last_time_).seconds();
        
        if (dt <= 0.0 || dt > 1.0) {
            RCLCPP_WARN_THROTTLE(this->node_->get_logger(), *this->node_->get_clock(), 5000,
                                "Invalid dt: %.6f seconds, skipping update", dt);
            last_time_ = current_time;
            return;
        }

        // Transform acceleration to world frame
        Eigen::Vector3d accel_world = orientation_ * accel_filtered;

        // Remove gravity if enabled
        if (remove_gravity_) {
            accel_world -= gravity_;
        }

        // Apply low-pass filter to acceleration
        acceleration_filtered_ = velocity_filter_alpha_ * accel_world + 
                                 (1.0 - velocity_filter_alpha_) * acceleration_filtered_;

        // Integrate acceleration to get velocity (with damping to reduce drift)
        Eigen::Vector3d velocity_raw = velocity_ + acceleration_filtered_ * dt;
        
        // Apply velocity damping to reduce drift (especially when stationary)
        double accel_magnitude = acceleration_filtered_.norm();
        if (accel_magnitude < 0.5) {  // Likely stationary or slow movement
            velocity_raw *= (1.0 - position_drift_compensation_);
        }

        // Low-pass filter on velocity
        velocity_ = velocity_filter_alpha_ * velocity_raw + 
                   (1.0 - velocity_filter_alpha_) * velocity_;

        // Integrate velocity to get position
        position_ += velocity_ * dt;

        // Z-axis constraint: maintain initial height when nearly stationary
        //if (std::abs(velocity_.z()) < 0.02 && accel_magnitude < 0.5) {
        //    // Gradually pull z back to initial height
        //    position_.z() = 0.99 * position_.z() + 0.01 * initial_height_;
        //}

        // Update last time
        last_time_ = current_time;

        // Publish odometry
        publishOdometry(current_time);
    }

    void publishOdometry(const rclcpp::Time& stamp) {
        nav_msgs::msg::Odometry odom_msg;
        
        odom_msg.header.stamp = stamp;
        odom_msg.header.frame_id = output_frame_id_;
        odom_msg.child_frame_id = child_frame_id_;

        // Position
        odom_msg.pose.pose.position.x = position_.x();
        odom_msg.pose.pose.position.y = position_.y();
        odom_msg.pose.pose.position.z = position_.z();

        // Orientation
        odom_msg.pose.pose.orientation.w = orientation_.w();
        odom_msg.pose.pose.orientation.x = orientation_.x();
        odom_msg.pose.pose.orientation.y = orientation_.y();
        odom_msg.pose.pose.orientation.z = orientation_.z();

        // Velocity (in child frame - transform from world frame)
        Eigen::Vector3d vel_body = orientation_.inverse() * velocity_;
        odom_msg.twist.twist.linear.x = vel_body.x();
        odom_msg.twist.twist.linear.y = vel_body.y();
        odom_msg.twist.twist.linear.z = vel_body.z();

        // Angular velocity (in body frame)
        odom_msg.twist.twist.angular.x = angular_velocity_.x();
        odom_msg.twist.twist.angular.y = angular_velocity_.y();
        odom_msg.twist.twist.angular.z = angular_velocity_.z();

        // Set covariances (indicating uncertainty due to drift)
        // Position covariance increases with time/integration
        double pos_cov = 0.02;  // Base uncertainty
        odom_msg.pose.covariance[0] = pos_cov;  // x
        odom_msg.pose.covariance[7] = pos_cov;  // y
        odom_msg.pose.covariance[14] = pos_cov * 0.3;  // z (better constrained)
        odom_msg.pose.covariance[21] = 0.01;  // roll
        odom_msg.pose.covariance[28] = 0.01;  // pitch
        odom_msg.pose.covariance[35] = 0.03;  // yaw (more uncertain without magnetometer)

        double vel_cov = 0.1;
        odom_msg.twist.covariance[0] = vel_cov;  // vx
        odom_msg.twist.covariance[7] = vel_cov;  // vy
        odom_msg.twist.covariance[14] = vel_cov;  // vz
        odom_msg.twist.covariance[21] = 0.05;  // wx
        odom_msg.twist.covariance[28] = 0.05;  // wy
        odom_msg.twist.covariance[35] = 0.05;  // wz

        odom_pub_->publish(odom_msg);
    }
};

} // namespace state_estimator_plugins

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(state_estimator_plugins::ImuOdomPlugin, state_estimator_plugins::PluginBase)
