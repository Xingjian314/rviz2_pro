#ifndef MAP_RENDER_WIDGET_HPP
#define MAP_RENDER_WIDGET_HPP

#include <QWidget>
#include <QTimer>
#include <QMouseEvent>
#include <QPainter>
#include <QPointF>

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include "path_planner.hpp"

class MapRenderWidget : public QWidget
{
  Q_OBJECT

public:
  MapRenderWidget(rclcpp::Node::SharedPtr node, QWidget *parent = nullptr);

protected:
  void paintEvent(QPaintEvent *event) override;
  void mousePressEvent(QMouseEvent *event) override;
  void resizeEvent(QResizeEvent *event) override;

private:
  void onTimer();
  void updateRobotPose();

  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
  rclcpp::Node::SharedPtr node_;
  QTimer *timer_;

  nav_msgs::msg::OccupancyGrid::SharedPtr map_;

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  QPointF robot_pos_;
  double robot_yaw_ = 0.0;
  bool robot_valid_ = false;

  QPointF goal_pos_;
  bool goal_valid_ = false;

  std::vector<QPointF> path_;

  PathPlanner planner_;

  // pixel <-> world mapping for current widget size
  float scale_ = 1.0f;
  float offset_x_ = 0.0f;
  float offset_y_ = 0.0f;

  void computeTransform();
  QPointF worldToPixel(float wx, float wy);
  QPointF pixelToWorld(int px, int py);
};

#endif
