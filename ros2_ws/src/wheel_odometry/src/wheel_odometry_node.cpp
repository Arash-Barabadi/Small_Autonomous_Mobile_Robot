// wheel_odometry_node.cpp
#include <cmath>
#include <cstdint>
#include <chrono>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32_multi_array.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"

#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/transform_broadcaster.h"

using namespace std::chrono_literals;

class WheelOdometryNode : public rclcpp::Node
{
public:
  WheelOdometryNode()
  : Node("wheel_odometry_node")
  {
    // ---------------------------------------------------------------
    // Parameters
    // ---------------------------------------------------------------
    wheel_radius_    = this->declare_parameter<double>("wheel_radius", 0.0335);
    track_width_     = this->declare_parameter<double>("track_width", 0.163);
    ticks_per_rev_   = this->declare_parameter<double>("ticks_per_rev", 585.0);

    // Always derived from wheel geometry and encoder resolution
    m_per_tick_ = (2.0 * M_PI * wheel_radius_) / ticks_per_rev_;

    odom_frame_      = this->declare_parameter<std::string>("odom_frame", "odom");
    base_frame_      = this->declare_parameter<std::string>("base_frame", "base_link");
    odom_topic_      = this->declare_parameter<std::string>("odom_topic", "odom");
    ticks_topic_     = this->declare_parameter<std::string>("ticks_topic", "wheel_ticks");

    // Set false only if another node (for example EKF) publishes odom -> base_link TF
    publish_tf_      = this->declare_parameter<bool>("publish_tf", true);
    publish_rate_hz_ = this->declare_parameter<double>("publish_rate_hz", 50.0);

    // If no new ticks arrive for this long, force reported velocities to zero
    vel_timeout_s_   = this->declare_parameter<double>("vel_timeout_s", 0.2);

    RCLCPP_INFO(this->get_logger(), "---------------------------------------");
    RCLCPP_INFO(this->get_logger(), "WheelOdometryNode parameters:");
    RCLCPP_INFO(this->get_logger(), "  wheel_radius   = %.5f m", wheel_radius_);
    RCLCPP_INFO(this->get_logger(), "  track_width    = %.5f m", track_width_);
    RCLCPP_INFO(this->get_logger(), "  ticks_per_rev  = %.1f", ticks_per_rev_);
    RCLCPP_INFO(this->get_logger(), "  m_per_tick     = %.8f m", m_per_tick_);
    RCLCPP_INFO(this->get_logger(), "  publish_tf     = %s", publish_tf_ ? "true" : "false");
    RCLCPP_INFO(this->get_logger(), "  publish_rate   = %.1f Hz", publish_rate_hz_);
    RCLCPP_INFO(this->get_logger(), "  vel_timeout    = %.2f s", vel_timeout_s_);
    RCLCPP_INFO(this->get_logger(), "  odom_frame     = %s", odom_frame_.c_str());
    RCLCPP_INFO(this->get_logger(), "  base_frame     = %s", base_frame_.c_str());
    RCLCPP_INFO(this->get_logger(), "  ticks_topic    = %s", ticks_topic_.c_str());
    RCLCPP_INFO(this->get_logger(), "  odom_topic     = %s", odom_topic_.c_str());
    RCLCPP_INFO(this->get_logger(), "---------------------------------------");

    // ---------------------------------------------------------------
    // Publisher
    // ---------------------------------------------------------------
    odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>(odom_topic_, 10);

    // ---------------------------------------------------------------
    // TF broadcaster
    // ---------------------------------------------------------------
    if (publish_tf_) {
      tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(*this);
    }

    // ---------------------------------------------------------------
    // Subscriber
    // ---------------------------------------------------------------
    ticks_sub_ = this->create_subscription<std_msgs::msg::Int32MultiArray>(
      ticks_topic_,
      50,
      std::bind(&WheelOdometryNode::ticksCallback, this, std::placeholders::_1)
    );

    // ---------------------------------------------------------------
    // Fixed-rate publish timer
    // ---------------------------------------------------------------
    if (publish_rate_hz_ < 1.0) {
      publish_rate_hz_ = 1.0;
    }

    auto period = std::chrono::duration<double>(1.0 / publish_rate_hz_);
    pub_timer_ = this->create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      std::bind(&WheelOdometryNode::publishTimer, this)
    );
  }

private:
  // ---------------------------------------------------------------
  // Tick subscriber callback
  // ---------------------------------------------------------------
  void ticksCallback(const std_msgs::msg::Int32MultiArray::SharedPtr msg)
  {
    if (msg->data.size() < 2) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(),
        *this->get_clock(),
        2000,
        "wheel_ticks message must contain at least 2 values [left, right]. Ignoring.");
      return;
    }

    const int32_t left_ticks  = static_cast<int32_t>(msg->data[0]);
    const int32_t right_ticks = static_cast<int32_t>(msg->data[1]);
    const rclcpp::Time now    = this->get_clock()->now();

    // First message initializes the baseline
    if (!have_prev_) {
      prev_left_ = left_ticks;
      prev_right_ = right_ticks;
      prev_time_ = now;
      last_tick_time_ = now;
      have_prev_ = true;

      RCLCPP_INFO(
        this->get_logger(),
        "First tick received. Odometry initialised at (0, 0, 0).");
      return;
    }

    // Guard against invalid or non-increasing time
    const double dt = (now - prev_time_).seconds();
    if (dt <= 1e-6) {
      prev_time_ = now;
      return;
    }

    // Encoder deltas
    const int32_t dL_ticks = left_ticks - prev_left_;
    const int32_t dR_ticks = right_ticks - prev_right_;

    prev_left_ = left_ticks;
    prev_right_ = right_ticks;
    prev_time_ = now;
    last_tick_time_ = now;

    // Convert ticks to metres
    const double dL = static_cast<double>(dL_ticks) * m_per_tick_;
    const double dR = static_cast<double>(dR_ticks) * m_per_tick_;

    // Differential-drive kinematics
    const double dS = 0.5 * (dL + dR);
    const double dTheta = (dR - dL) / track_width_;

    // Midpoint integration
    const double theta_mid = theta_ + 0.5 * dTheta;
    x_ += dS * std::cos(theta_mid);
    y_ += dS * std::sin(theta_mid);
    theta_ = normalizeAngle(theta_ + dTheta);

    // Velocities for odom twist
    last_v_ = dS / dt;
    last_w_ = dTheta / dt;
  }

  // ---------------------------------------------------------------
  // Fixed-rate publish timer
  // ---------------------------------------------------------------
  void publishTimer()
  {
    if (!have_prev_) {
      return;
    }

    const rclcpp::Time stamp = this->get_clock()->now();

    // Zero velocity if no fresh encoder update has arrived recently
    const double tick_age = (stamp - last_tick_time_).seconds();
    if (tick_age > vel_timeout_s_) {
      last_v_ = 0.0;
      last_w_ = 0.0;
    }

    publishOdomAndTF(stamp, last_v_, last_w_);
  }

  // ---------------------------------------------------------------
  // Publish odometry and optional TF
  // ---------------------------------------------------------------
  void publishOdomAndTF(const rclcpp::Time & stamp, double v, double w)
  {
    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, theta_);
    q.normalize();

    nav_msgs::msg::Odometry odom;
    odom.header.stamp = stamp;
    odom.header.frame_id = odom_frame_;
    odom.child_frame_id = base_frame_;

    odom.pose.pose.position.x = x_;
    odom.pose.pose.position.y = y_;
    odom.pose.pose.position.z = 0.0;
    odom.pose.pose.orientation.x = q.x();
    odom.pose.pose.orientation.y = q.y();
    odom.pose.pose.orientation.z = q.z();
    odom.pose.pose.orientation.w = q.w();

    odom.twist.twist.linear.x = v;
    odom.twist.twist.linear.y = 0.0;
    odom.twist.twist.angular.z = w;

    for (auto & c : odom.pose.covariance) {
      c = 0.0;
    }
    for (auto & c : odom.twist.covariance) {
      c = 0.0;
    }

    // Pose covariance
    odom.pose.covariance[0]  = 0.05;  // x
    odom.pose.covariance[7]  = 0.05;  // y
    odom.pose.covariance[35] = 0.10;  // yaw

    // Twist covariance
    odom.twist.covariance[0]  = 0.10; // vx
    odom.twist.covariance[35] = 0.15; // vyaw

    odom_pub_->publish(odom);

    if (publish_tf_ && tf_broadcaster_) {
      geometry_msgs::msg::TransformStamped tf_msg;
      tf_msg.header.stamp = stamp;
      tf_msg.header.frame_id = odom_frame_;
      tf_msg.child_frame_id = base_frame_;

      tf_msg.transform.translation.x = x_;
      tf_msg.transform.translation.y = y_;
      tf_msg.transform.translation.z = 0.0;
      tf_msg.transform.rotation.x = q.x();
      tf_msg.transform.rotation.y = q.y();
      tf_msg.transform.rotation.z = q.z();
      tf_msg.transform.rotation.w = q.w();

      tf_broadcaster_->sendTransform(tf_msg);
    }
  }

  // ---------------------------------------------------------------
  // Utility
  // ---------------------------------------------------------------
  static double normalizeAngle(double a)
  {
    while (a > M_PI) {
      a -= 2.0 * M_PI;
    }
    while (a < -M_PI) {
      a += 2.0 * M_PI;
    }
    return a;
  }

  // ---------------------------------------------------------------
  // Parameters
  // ---------------------------------------------------------------
  double wheel_radius_{0.0335};
  double track_width_{0.163};
  double ticks_per_rev_{585.0};
  double m_per_tick_{0.0};

  std::string odom_frame_{"odom"};
  std::string base_frame_{"base_link"};
  std::string odom_topic_{"odom"};
  std::string ticks_topic_{"wheel_ticks"};

  bool publish_tf_{true};
  double publish_rate_hz_{50.0};
  double vel_timeout_s_{0.2};

  // ---------------------------------------------------------------
  // ROS handles
  // ---------------------------------------------------------------
  rclcpp::Subscription<std_msgs::msg::Int32MultiArray>::SharedPtr ticks_sub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  rclcpp::TimerBase::SharedPtr pub_timer_;

  // ---------------------------------------------------------------
  // State
  // ---------------------------------------------------------------
  bool have_prev_{false};
  int32_t prev_left_{0};
  int32_t prev_right_{0};

  rclcpp::Time prev_time_;
  rclcpp::Time last_tick_time_;

  double x_{0.0};
  double y_{0.0};
  double theta_{0.0};

  double last_v_{0.0};
  double last_w_{0.0};
};

// ---------------------------------------------------------------
// Main
// ---------------------------------------------------------------
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<WheelOdometryNode>());
  rclcpp::shutdown();
  return 0;
}
