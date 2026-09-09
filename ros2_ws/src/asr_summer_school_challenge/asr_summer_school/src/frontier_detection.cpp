#include "asr_summer_school/frontier_detection.h"

#include <cmath>
#include <limits>
#include <map>
#include <queue>
#include <vector>
#include <utility>

namespace
{

std::vector<std::pair<int, int>> get_neighbors(
  const nav_msgs::msg::OccupancyGrid & grid,
  double epsilon,
  int x, int y)
{
  std::vector<std::pair<int, int>> neighbors;

  const int height_cells = static_cast<int>(grid.info.height);
  const int width_cells  = static_cast<int>(grid.info.width);
  const double res       = grid.info.resolution;
  const int eps_cells    = static_cast<int>(std::ceil(epsilon / res));

  for (int dx = -eps_cells; dx <= eps_cells; ++dx) {
    for (int dy = -eps_cells; dy <= eps_cells; ++dy) {
      if (dx == 0 && dy == 0) continue;
      if ((dx * dx + dy * dy) * res * res > epsilon * epsilon) continue;

      const int nx = x + dx;
      const int ny = y + dy;

      if (nx >= 0 && ny >= 0 && nx < width_cells && ny < height_cells &&
          grid.data[ny * width_cells + nx] == 100)
      {
        neighbors.emplace_back(nx, ny);
      }
    }
  }

  return neighbors;
}

std::map<int, std::vector<std::pair<int, int>>> dbscan(
  const nav_msgs::msg::OccupancyGrid & grid,
  const frontier_detection::Params & params)
{
  std::map<std::pair<int, int>, int> labels;
  std::map<int, std::vector<std::pair<int, int>>> clusters;

  int cluster_id = 0;
  const int height_cells = static_cast<int>(grid.info.height);
  const int width_cells  = static_cast<int>(grid.info.width);

  for (int y = 0; y < height_cells; ++y) {
    for (int x = 0; x < width_cells; ++x) {
      if (grid.data[y * width_cells + x] != 100) continue;
      if (labels.count({x, y})) continue;

      auto neighbors = get_neighbors(grid, params.epsilon, x, y);

      if (static_cast<int>(neighbors.size()) < params.min_points) {
        labels[{x, y}] = -1;
        continue;
      }

      labels[{x, y}] = cluster_id;
      clusters[cluster_id].push_back({x, y});

      std::queue<std::pair<int, int>> q;
      for (const auto & p : neighbors) q.push(p);

      while (!q.empty()) {
        auto p = q.front(); q.pop();

        if (labels.count(p)) {
          if (labels[p] == -1) {
            labels[p] = cluster_id;
            clusters[cluster_id].push_back(p);
          }
          continue;
        }

        labels[p] = cluster_id;
        clusters[cluster_id].push_back(p);

        auto p_neighbors = get_neighbors(grid, params.epsilon, p.first, p.second);
        if (static_cast<int>(p_neighbors.size()) >= params.min_points) {
          for (const auto & n : p_neighbors) q.push(n);
        }
      }

      ++cluster_id;
    }
  }

  return clusters;
}

std::vector<std::pair<double, double>> compute_centroids(
  const nav_msgs::msg::OccupancyGrid & grid,
  const std::map<int, std::vector<std::pair<int, int>>> & clusters,
  int min_frontier_size)
{
  std::vector<std::pair<double, double>> centroids;

  const double res      = grid.info.resolution;
  const double origin_x = grid.info.origin.position.x;
  const double origin_y = grid.info.origin.position.y;

  for (const auto & [id, cluster] : clusters) {
    if (static_cast<int>(cluster.size()) < min_frontier_size) continue;

    double sum_x = 0.0, sum_y = 0.0;
    for (const auto & pt : cluster) { sum_x += pt.first; sum_y += pt.second; }

    const double cx = sum_x / cluster.size();
    const double cy = sum_y / cluster.size();

    // Use the cluster point closest to the geometric centroid
    std::pair<int, int> closest;
    double min_dist_sq = std::numeric_limits<double>::max();
    for (const auto & pt : cluster) {
      const double dx = pt.first  - cx;
      const double dy = pt.second - cy;
      const double d  = dx * dx + dy * dy;
      if (d < min_dist_sq) { min_dist_sq = d; closest = pt; }
    }

    centroids.emplace_back(
      closest.first  * res + origin_x + res / 2.0,
      closest.second * res + origin_y + res / 2.0);
  }

  return centroids;
}

}  // anonymous namespace

namespace frontier_detection
{

nav_msgs::msg::OccupancyGrid preprocess_frontier_cells(
  const nav_msgs::msg::OccupancyGrid & map_grid,
  double robot_world_x,
  double robot_world_y,
  double active_area_radius)
{
  const int width_cells  = static_cast<int>(map_grid.info.width);
  const int height_cells = static_cast<int>(map_grid.info.height);
  const double res       = map_grid.info.resolution;

  const int rx = static_cast<int>((robot_world_x - map_grid.info.origin.position.x) / res);
  const int ry = static_cast<int>((robot_world_y - map_grid.info.origin.position.y) / res);

  nav_msgs::msg::OccupancyGrid frontier_grid = map_grid;
  std::fill(frontier_grid.data.begin(), frontier_grid.data.end(), -1);

  if (rx < 0 || rx >= width_cells || ry < 0 || ry >= height_cells)
    return frontier_grid;

  const int dx_table[4] = {0, 1, 0, -1};
  const int dy_table[4] = {1, 0, -1, 0};
  const double sq_radius = active_area_radius * active_area_radius;

  std::vector<bool> visited(static_cast<size_t>(width_cells * height_cells), false);
  std::queue<int> queue;
  const int robot_index = ry * width_cells + rx;
  queue.push(robot_index);
  visited[robot_index] = true;

  while (!queue.empty()) {
    const int index = queue.front();
    queue.pop();

    if (map_grid.data[index] != 0)
      continue;

    const int x = index % width_cells;
    const int y = index / width_cells;
    bool is_frontier = false;

    for (int i = 0; i < 4; ++i) {
      const int nx = x + dx_table[i];
      const int ny = y + dy_table[i];

      if (nx < 0 || nx >= width_cells || ny < 0 || ny >= height_cells)
        continue;

      const int nindex = ny * width_cells + nx;
      if (visited[nindex])
        continue;

      const double nx_dist = (nx - rx) * res;
      const double ny_dist = (ny - ry) * res;
      if (nx_dist * nx_dist + ny_dist * ny_dist > sq_radius)
        continue;

      if (map_grid.data[nindex] == -1) {
        is_frontier = true;
      } else if (map_grid.data[nindex] == 0) {
        queue.push(nindex);
        visited[nindex] = true;
      }
    }

    frontier_grid.data[index] = is_frontier ? 100 : -1;
  }

  return frontier_grid;
}

std::vector<std::pair<double, double>> detect_frontiers(
  const nav_msgs::msg::OccupancyGrid & grid,
  const Params & params,
  double robot_x,
  double robot_y)
{
  auto frontier_grid = preprocess_frontier_cells(grid, robot_x, robot_y, params.active_area_radius);
  auto clusters = dbscan(frontier_grid, params);
  return compute_centroids(frontier_grid, clusters, params.min_frontier_size);
}

void publish_frontiers_marker(
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr publisher,
  const std::vector<std::pair<double, double>> & centroids,
  const std::string & frame_id,
  rclcpp::Clock::SharedPtr clock)
{
  visualization_msgs::msg::Marker marker;
  marker.header.frame_id = frame_id;
  marker.header.stamp    = clock->now();
  marker.ns              = "frontiers";
  marker.id              = 0;
  marker.type            = visualization_msgs::msg::Marker::POINTS;
  marker.action          = visualization_msgs::msg::Marker::ADD;
  marker.color.r         = 0.1f;
  marker.color.g         = 0.4f;
  marker.color.b         = 0.1f;
  marker.color.a         = 1.0f;
  marker.scale.x         = 0.2;
  marker.scale.y         = 0.2;

  for (const auto & c : centroids) {
    geometry_msgs::msg::Point p;
    p.x = c.first;
    p.y = c.second;
    p.z = 0.0;
    marker.points.push_back(p);
  }

  publisher->publish(marker);
}

}  // namespace frontier_detection
