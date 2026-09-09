#ifndef ASR_SUMMER_SCHOOL__FRONTIER_DETECTION_H
#define ASR_SUMMER_SCHOOL__FRONTIER_DETECTION_H

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <geometry_msgs/msg/point.hpp>

#include <vector>
#include <utility>

namespace frontier_detection
{

struct Params
{
  double epsilon;              // DBSCAN neighborhood radius in meters
  int min_points;              // DBSCAN minimum neighbors to form a core point
  int min_frontier_size;       // minimum cluster size to be a valid frontier
  double active_area_radius;   // wavefront expansion radius in meters
};

/// Marks free cells adjacent to unknown space within active_area_radius of the robot as frontier (100).
nav_msgs::msg::OccupancyGrid preprocess_frontier_cells(
  const nav_msgs::msg::OccupancyGrid & map_grid,
  double robot_world_x,
  double robot_world_y,
  double active_area_radius);

/// Preprocesses the grid for frontiers then runs DBSCAN; returns world-frame centroids.
std::vector<std::pair<double, double>> detect_frontiers(
  const nav_msgs::msg::OccupancyGrid & grid,
  const Params & params,
  double robot_x,
  double robot_y);

/// Publishes frontier centroids as a POINTS marker.
void publish_frontiers_marker(
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr publisher,
  const std::vector<std::pair<double, double>> & centroids,
  const std::string & frame_id,
  rclcpp::Clock::SharedPtr clock);

}  // namespace frontier_detection

#endif  // ASR_SUMMER_SCHOOL__FRONTIER_DETECTION_H
