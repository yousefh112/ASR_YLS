#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <visualization_msgs/msg/marker.hpp>

#include "asr_summer_school/frontier_detection.h"

class FrontierDetectionNode : public rclcpp::Node
{
public:
  explicit FrontierDetectionNode(const rclcpp::NodeOptions & options)
  : Node("frontier_detection_node", options)
  {
    declare_parameter("epsilon", 0.5);
    declare_parameter("min_points", 3);
    declare_parameter("min_frontier_size", 5);
    // Matches the LDS-02 horizon. Above this the detector proposes
    // frontiers the robot can never observe and exploration stalls
    // chasing them; the launch files override it with laser_max_range.
    declare_parameter("active_area_radius", 3.5);
    declare_parameter("map_topic", std::string("map"));
    // This stack runs slam_toolbox, not AMCL: slam_toolbox publishes the
    // robot pose on "pose". With the old "amcl_pose" default, running this
    // node outside the launch files leaves robot_x_/robot_y_ at (0, 0) and
    // the active area centred on the map origin.
    declare_parameter("pose_topic", std::string("pose"));

    params_.epsilon             = get_parameter("epsilon").as_double();
    params_.min_points          = get_parameter("min_points").as_int();
    params_.min_frontier_size   = get_parameter("min_frontier_size").as_int();
    params_.active_area_radius  = get_parameter("active_area_radius").as_double();

    marker_pub_ = create_publisher<visualization_msgs::msg::Marker>(
      "frontier_centroids", rclcpp::QoS(1).transient_local());

    pose_sub_ = create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
      get_parameter("pose_topic").as_string(), rclcpp::QoS(1),
      [this](const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg) {
        robot_x_ = msg->pose.pose.position.x;
        robot_y_ = msg->pose.pose.position.y;
      });

    map_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
      get_parameter("map_topic").as_string(), rclcpp::QoS(1).transient_local(),
      [this](const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
        auto centroids = frontier_detection::detect_frontiers(*msg, params_, robot_x_, robot_y_);
        frontier_detection::publish_frontiers_marker(
          marker_pub_, centroids, msg->header.frame_id, get_clock());
      });
  }

private:
  frontier_detection::Params params_;
  double robot_x_{0.0};
  double robot_y_{0.0};
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_sub_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
};

RCLCPP_COMPONENTS_REGISTER_NODE(FrontierDetectionNode)
