#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <laser_geometry/laser_geometry.hpp>

class LaserToPointCloud : public rclcpp::Node
{
  laser_geometry::LaserProjection projector_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr sub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_;

public:
  LaserToPointCloud() : Node("laser_to_pointcloud")
  {
    sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
      "/scan", 10,
      [this](sensor_msgs::msg::LaserScan::ConstSharedPtr scan) {
        auto cloud = std::make_shared<sensor_msgs::msg::PointCloud2>();
        projector_.projectLaser(*scan, *cloud);
        pub_->publish(*cloud);
      });

    pub_ = create_publisher<sensor_msgs::msg::PointCloud2>("/pointcloud", 10);
  }
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LaserToPointCloud>());
  rclcpp::shutdown();
  return 0;
}
