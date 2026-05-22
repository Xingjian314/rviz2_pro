#include <QApplication>
#include <rclcpp/rclcpp.hpp>

#include "pointcloud_widget.hpp"

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("pointcloud_viewer");

  QApplication app(argc, argv);

  PointCloudWidget widget(node);
  widget.resize(800, 600);
  widget.setWindowTitle("PointCloud Viewer");
  widget.show();

  int ret = app.exec();
  rclcpp::shutdown();
  return ret;
}
