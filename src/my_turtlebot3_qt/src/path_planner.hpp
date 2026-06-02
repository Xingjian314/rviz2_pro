#ifndef PATH_PLANNER_HPP
#define PATH_PLANNER_HPP

#include <vector>
#include <QPointF>
#include <nav_msgs/msg/occupancy_grid.hpp>

struct AStarNode
{
  int x, y;
  float g, h, f;
  AStarNode *parent;
};

class PathPlanner
{
public:
  std::vector<QPointF> findPath(
    const nav_msgs::msg::OccupancyGrid &map,
    float start_wx, float start_wy,
    float goal_wx, float goal_wy);

private:
  void worldToGrid(const nav_msgs::msg::OccupancyGrid &map,
    float wx, float wy, int &gx, int &gy);
  QPointF gridToWorld(const nav_msgs::msg::OccupancyGrid &map,
    int gx, int gy);
};

#endif
