#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

class ScanRestamper : public rclcpp::Node
{
public:
  ScanRestamper() : Node("scan_restamper")
  {
    sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
      "/scan_raw", rclcpp::SensorDataQoS(),
      [this](sensor_msgs::msg::LaserScan::SharedPtr msg) {
        msg->header.stamp = this->get_clock()->now() - rclcpp::Duration::from_seconds(0.05);
        pub_->publish(*msg);
      });

    pub_ = this->create_publisher<sensor_msgs::msg::LaserScan>(
      "/scan", rclcpp::SensorDataQoS());

    RCLCPP_INFO(this->get_logger(), "ScanRestamper ready: /scan_raw -> /scan");
  }

private:
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr sub_;
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr    pub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ScanRestamper>());
  rclcpp::shutdown();
  return 0;
}

