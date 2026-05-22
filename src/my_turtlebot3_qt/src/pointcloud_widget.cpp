#include "pointcloud_widget.hpp"

#include <GL/gl.h>
#include <GL/glu.h>

PointCloudWidget::PointCloudWidget(rclcpp::Node::SharedPtr node, QWidget *parent)
  : QOpenGLWidget(parent), node_(node)
{
  sub_ = node_->create_subscription<sensor_msgs::msg::PointCloud2>(
    "/pointcloud", 10,
    [this](sensor_msgs::msg::PointCloud2::ConstSharedPtr msg) {
      parsePointCloud(*msg);
      update();
    });

  timer_ = new QTimer(this);
  connect(timer_, &QTimer::timeout, [this]() {
    rclcpp::spin_some(node_);
  });
  timer_->start(16);
}

void PointCloudWidget::parsePointCloud(const sensor_msgs::msg::PointCloud2 &msg)
{
  int offset_x = -1, offset_y = -1, offset_z = -1;
  for (const auto &field : msg.fields) {
    if (field.name == "x") offset_x = field.offset;
    if (field.name == "y") offset_y = field.offset;
    if (field.name == "z") offset_z = field.offset;
  }
  if (offset_x < 0 || offset_y < 0 || offset_z < 0)
    return;

  points_.clear();
  size_t point_step = msg.point_step;
  size_t n = msg.width * msg.height;
  points_.reserve(n * 3);

  const uint8_t *data = msg.data.data();
  for (size_t i = 0; i < n; ++i) {
    const uint8_t *p = data + i * point_step;
    float x, y, z;
    memcpy(&x, p + offset_x, sizeof(float));
    memcpy(&y, p + offset_y, sizeof(float));
    memcpy(&z, p + offset_z, sizeof(float));
    points_.push_back(x);
    points_.push_back(y);
    points_.push_back(z);
  }
  new_data_ = true;
}

void PointCloudWidget::initializeGL()
{
  initializeOpenGLFunctions();
  glClearColor(0.1f, 0.1f, 0.1f, 1.0f);
  glEnable(GL_DEPTH_TEST);
  glPointSize(3.0f);
}

void PointCloudWidget::resizeGL(int w, int h)
{
  glViewport(0, 0, w, h);
}

void PointCloudWidget::paintGL()
{
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

  glMatrixMode(GL_PROJECTION);
  glLoadIdentity();
  float aspect = static_cast<float>(width()) / std::max(height(), 1);
  gluPerspective(45.0, aspect, 0.1, 100.0);

  glMatrixMode(GL_MODELVIEW);
  glLoadIdentity();
  glTranslatef(0.0f, 0.0f, zoom_);
  glRotatef(rot_x_, 1.0f, 0.0f, 0.0f);
  glRotatef(rot_y_, 0.0f, 1.0f, 0.0f);
  glRotatef(rot_z_, 0.0f, 0.0f, 1.0f);

  glBegin(GL_POINTS);
  glColor3f(0.0f, 1.0f, 0.0f);
  for (size_t i = 0; i < points_.size(); i += 3) {
    glVertex3f(points_[i], points_[i + 1], points_[i + 2]);
  }
  glEnd();

  // Draw axes
  glBegin(GL_LINES);
  glColor3f(1, 0, 0); glVertex3f(0,0,0); glVertex3f(1,0,0);
  glColor3f(0, 1, 0); glVertex3f(0,0,0); glVertex3f(0,1,0);
  glColor3f(0, 0, 1); glVertex3f(0,0,0); glVertex3f(0,0,1);
  glEnd();
}

void PointCloudWidget::mousePressEvent(QMouseEvent *event)
{
  last_mouse_pos_ = event->pos();
}

void PointCloudWidget::mouseMoveEvent(QMouseEvent *event)
{
  int dx = event->pos().x() - last_mouse_pos_.x();
  int dy = event->pos().y() - last_mouse_pos_.y();

  if (event->buttons() & Qt::LeftButton) {
    rot_y_ += dx * 0.5f;
    rot_x_ += dy * 0.5f;
    update();
  } else if (event->buttons() & Qt::RightButton) {
    rot_z_ += dx * 0.5f;
    update();
  }
  last_mouse_pos_ = event->pos();
}

void PointCloudWidget::wheelEvent(QWheelEvent *event)
{
  zoom_ += event->angleDelta().y() * 0.005f;
  update();
}
