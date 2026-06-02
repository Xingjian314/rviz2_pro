#include "map_render_widget.hpp"

#include <cmath>
#include <QPainter>
#include <QMouseEvent>

MapRenderWidget::MapRenderWidget(rclcpp::Node::SharedPtr node, QWidget *parent)
  : QWidget(parent), node_(node),
    tf_buffer_(node_->get_clock()),
    tf_listener_(tf_buffer_, node, true)
{
  auto qos = rclcpp::QoS(1).transient_local();
  map_sub_ = node_->create_subscription<nav_msgs::msg::OccupancyGrid>(
    "/map", qos,
    [this](nav_msgs::msg::OccupancyGrid::ConstSharedPtr msg) {
      map_ = std::make_shared<nav_msgs::msg::OccupancyGrid>(*msg);
      computeTransform();
      update();
    });

  timer_ = new QTimer(this);
  connect(timer_, &QTimer::timeout, this, &MapRenderWidget::onTimer);
  timer_->start(33);
}

void MapRenderWidget::onTimer()
{
  rclcpp::spin_some(node_);
  updateRobotPose();
  update();
}

void MapRenderWidget::updateRobotPose()
{
  geometry_msgs::msg::TransformStamped ts;
  try {
    ts = tf_buffer_.lookupTransform("map", "base_footprint", tf2::TimePointZero);
  } catch (tf2::TransformException &) {
    return;
  }
  robot_pos_.setX(ts.transform.translation.x);
  robot_pos_.setY(ts.transform.translation.y);
  robot_yaw_ = std::atan2(
    2.0 * (ts.transform.rotation.z * ts.transform.rotation.w +
           ts.transform.rotation.x * ts.transform.rotation.y),
    1.0 - 2.0 * (ts.transform.rotation.y * ts.transform.rotation.y +
                 ts.transform.rotation.z * ts.transform.rotation.z));
  robot_valid_ = true;
}

void MapRenderWidget::computeTransform()
{
  if (!map_) return;
  float mw = map_->info.width;
  float mh = map_->info.height;
  float res = map_->info.resolution;
  float map_pixel_w = mw;
  float map_pixel_h = mh;
  float widget_w = width();
  float widget_h = height();

  scale_ = std::min(widget_w / map_pixel_w, widget_h / map_pixel_h) * 0.9f;
  offset_x_ = (widget_w - map_pixel_w * scale_) / 2.0f;
  offset_y_ = (widget_h - map_pixel_h * scale_) / 2.0f;
}

QPointF MapRenderWidget::worldToPixel(float wx, float wy)
{
  if (!map_) return {};
  float ox = map_->info.origin.position.x;
  float oy = map_->info.origin.position.y;
  float res = map_->info.resolution;
  float px = (wx - ox) / res;
  float py = (wy - oy) / res;
  return QPointF(px * scale_ + offset_x_, py * scale_ + offset_y_);
}

QPointF MapRenderWidget::pixelToWorld(int px, int py)
{
  if (!map_) return {};
  float ox = map_->info.origin.position.x;
  float oy = map_->info.origin.position.y;
  float res = map_->info.resolution;
  float gx = (px - offset_x_) / scale_;
  float gy = (py - offset_y_) / scale_;
  return QPointF(gx * res + ox, gy * res + oy);
}

void MapRenderWidget::resizeEvent(QResizeEvent *)
{
  computeTransform();
}

void MapRenderWidget::paintEvent(QPaintEvent *)
{
  QPainter p(this);
  p.setRenderHint(QPainter::Antialiasing);

  // Background
  p.fillRect(rect(), QColor(40, 40, 40));

  if (!map_) {
    p.setPen(Qt::white);
    p.drawText(rect(), Qt::AlignCenter, "Waiting for /map...");
    return;
  }

  // Draw map as grayscale image
  int w = map_->info.width;
  int h = map_->info.height;
  QImage img(w, h, QImage::Format_Grayscale8);
  for (int y = 0; y < h; ++y) {
    for (int x = 0; x < w; ++x) {
      int val = map_->data[y * w + x];
      uchar gray;
      if (val < 0)
        gray = 100;   // unknown
      else if (val > 50)
        gray = 0;     // occupied
      else
        gray = 255;   // free
      img.setPixel(x, y, qRgb(gray, gray, gray));
    }
  }

  QRectF target(offset_x_, offset_y_, w * scale_, h * scale_);
  p.drawImage(target, img);

  // Draw path
  if (!path_.empty()) {
    QPen pen(QColor(0, 120, 255), 3);
    p.setPen(pen);
    for (size_t i = 0; i < path_.size() - 1; ++i) {
      auto a = worldToPixel(path_[i].x(), path_[i].y());
      auto b = worldToPixel(path_[i + 1].x(), path_[i + 1].y());
      p.drawLine(a, b);
    }
  }

  // Draw goal
  if (goal_valid_) {
    auto gp = worldToPixel(goal_pos_.x(), goal_pos_.y());
    p.setPen(Qt::green);
    p.setBrush(Qt::green);
    p.drawEllipse(gp, 6, 6);
    p.drawText(gp + QPointF(10, 5), "Goal");
  }

  // Draw robot
  if (robot_valid_) {
    auto rp = worldToPixel(robot_pos_.x(), robot_pos_.y());
    p.setPen(Qt::red);
    p.setBrush(Qt::red);
    p.drawEllipse(rp, 8, 8);

    // Orientation arrow
    float arrow_len = 20;
    float ex = rp.x() + arrow_len * std::cos(robot_yaw_);
    float ey = rp.y() + arrow_len * std::sin(robot_yaw_);
    p.drawLine(rp, QPointF(ex, ey));

    // Circle around robot
    p.setPen(QPen(Qt::red, 2));
    p.setBrush(Qt::NoBrush);
    p.drawEllipse(rp, 15, 15);
  }
}

void MapRenderWidget::mousePressEvent(QMouseEvent *event)
{
  if (!map_ || event->button() != Qt::LeftButton) return;

  auto world = pixelToWorld(event->pos().x(), event->pos().y());
  goal_pos_ = world;
  goal_valid_ = true;

  if (robot_valid_) {
    path_ = planner_.findPath(*map_,
      robot_pos_.x(), robot_pos_.y(),
      goal_pos_.x(), goal_pos_.y());
  }

  update();
}
