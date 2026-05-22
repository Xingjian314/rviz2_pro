#ifndef POINTCLOUD_WIDGET_HPP
#define POINTCLOUD_WIDGET_HPP

#include <QOpenGLWidget>
#include <QOpenGLFunctions>
#include <QTimer>
#include <QMouseEvent>  
#include <QWheelEvent>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

class PointCloudWidget : public QOpenGLWidget, protected QOpenGLFunctions
{
  Q_OBJECT

public:
  PointCloudWidget(rclcpp::Node::SharedPtr node, QWidget *parent = nullptr);
  ~PointCloudWidget() override = default;

protected:
  void initializeGL() override;
  void paintGL() override;
  void resizeGL(int w, int h) override;
  void mousePressEvent(QMouseEvent *event) override;
  void mouseMoveEvent(QMouseEvent *event) override;
  void wheelEvent(QWheelEvent *event) override;

private:
  void parsePointCloud(const sensor_msgs::msg::PointCloud2 &msg);

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;
  rclcpp::Node::SharedPtr node_;
  QTimer *timer_;

  std::vector<float> points_;
  bool new_data_ = false;

  float rot_x_ = 0.0f;
  float rot_y_ = 0.25f;
  float rot_z_ = 0.0f;
  float zoom_ = -3.0f;
  QPoint last_mouse_pos_;
};

#endif
