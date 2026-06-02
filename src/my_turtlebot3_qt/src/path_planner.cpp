#include "path_planner.hpp"

#include <queue>
#include <unordered_set>
#include <cmath>
#include <algorithm>
#include <limits>

void PathPlanner::worldToGrid(const nav_msgs::msg::OccupancyGrid &map,
  float wx, float wy, int &gx, int &gy)
{
  float ox = map.info.origin.position.x;
  float oy = map.info.origin.position.y;
  float res = map.info.resolution;
  gx = static_cast<int>((wx - ox) / res);
  gy = static_cast<int>((wy - oy) / res);
}

QPointF PathPlanner::gridToWorld(const nav_msgs::msg::OccupancyGrid &map,
  int gx, int gy)
{
  float ox = map.info.origin.position.x;
  float oy = map.info.origin.position.y;
  float res = map.info.resolution;
  return QPointF(gx * res + ox, gy * res + oy);
}

std::vector<QPointF> PathPlanner::findPath(
  const nav_msgs::msg::OccupancyGrid &map,
  float start_wx, float start_wy,
  float goal_wx, float goal_wy)
{
  int sx, sy, gx, gy;
  worldToGrid(map, start_wx, start_wy, sx, sy);
  worldToGrid(map, goal_wx, goal_wy, gx, gy);

  int w = map.info.width;
  int h = map.info.height;

  if (sx < 0 || sx >= w || sy < 0 || sy >= h ||
      gx < 0 || gx >= w || gy < 0 || gy >= h)
    return {};

  if (map.data[sy * w + sx] > 50 || map.data[gy * w + gx] > 50)
    return {};

  auto idx = [w](int x, int y) { return y * w + x; };

  std::vector<float> g_cost(w * h, std::numeric_limits<float>::max());
  std::vector<AStarNode> nodes(w * h);

  auto cmp = [](const AStarNode *a, const AStarNode *b) { return a->f > b->f; };
  std::priority_queue<AStarNode *, std::vector<AStarNode *>, decltype(cmp)> open(cmp);
  std::vector<bool> closed(w * h, false);

  nodes[idx(sx, sy)] = {sx, sy, 0.0f, 0.0f, 0.0f, nullptr};
  g_cost[idx(sx, sy)] = 0;
  open.push(&nodes[idx(sx, sy)]);

  const int dirs[8][2] = {{1,0},{-1,0},{0,1},{0,-1},{1,1},{1,-1},{-1,1},{-1,-1}};

  while (!open.empty()) {
    auto current = open.top();
    open.pop();

    if (closed[idx(current->x, current->y)])
      continue;
    closed[idx(current->x, current->y)] = true;

    if (current->x == gx && current->y == gy) {
      std::vector<QPointF> path;
      auto *n = current;
      while (n) {
        path.push_back(gridToWorld(map, n->x, n->y));
        n = n->parent;
      }
      std::reverse(path.begin(), path.end());
      return path;
    }

    for (auto &d : dirs) {
      int nx = current->x + d[0];
      int ny = current->y + d[1];
      if (nx < 0 || nx >= w || ny < 0 || ny >= h)
        continue;
      if (closed[idx(nx, ny)])
        continue;
      if (map.data[ny * w + nx] > 50)
        continue;

      float step = (d[0] != 0 && d[1] != 0) ? 1.414f : 1.0f;
      float new_g = current->g + step;
      if (new_g >= g_cost[idx(nx, ny)])
        continue;

      float dx = nx - gx, dy = ny - gy;
      float heur = std::sqrt(dx * dx + dy * dy);

      nodes[idx(nx, ny)] = {nx, ny, new_g, heur, new_g + heur, current};
      g_cost[idx(nx, ny)] = new_g;
      open.push(&nodes[idx(nx, ny)]);
    }
  }

  return {};
}
