#include <QApplication>
#include <QSplitter>
#include <rclcpp/rclcpp.hpp>

#include "pointcloud_widget.hpp"
#include "map_render_widget.hpp"

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("pointcloud_viewer");

  QApplication app(argc, argv);

  auto splitter = new QSplitter(Qt::Vertical);

  auto map_widget = new MapRenderWidget(node);
  auto cloud_widget = new PointCloudWidget(node);

  splitter->addWidget(map_widget);
  splitter->addWidget(cloud_widget);
  splitter->resize(800, 700);
  splitter->setStretchFactor(0, 1);
  splitter->setStretchFactor(1, 1);
  splitter->setWindowTitle("TurtleBot3 Qt Viewer");

  splitter->show();

  int ret = app.exec();
  rclcpp::shutdown();
  return ret;
}
