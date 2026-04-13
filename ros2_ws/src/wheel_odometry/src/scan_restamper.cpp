#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

class ScanRestamper : public rclcpp::Node
{
public:
  ScanRestamper() : Node("scan_restamper")
  {
    auto qos = rclcpp::QoS(rclcpp::KeepLast(10)).reliable();

    sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
      "/scan_raw", qos,
      [this](sensor_msgs::msg::LaserScan::SharedPtr msg) {
        auto now = this->get_clock()->now();
        auto msg_time = rclcpp::Time(msg->header.stamp);
        auto age = now - msg_time;

        // Drop obviously stale scans before restamping
        if (age > rclcpp::Duration::from_seconds(0.3)) {
          RCLCPP_WARN_THROTTLE(
            this->get_logger(),
            *this->get_clock(),
            2000,
            "Dropping stale scan (age too large before restamp).");
          return;
        }

        msg->header.stamp = now - rclcpp::Duration::from_seconds(0.05);
        pub_->publish(*msg);
      });

    pub_ = this->create_publisher<sensor_msgs::msg::LaserScan>(
      "/scan", qos);

    RCLCPP_INFO(this->get_logger(), "ScanRestamper ready: /scan_raw -> /scan");
  }

private:
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr sub_;
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr pub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ScanRestamper>());
  rclcpp::shutdown();
  return 0;
}
