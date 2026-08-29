#include "rrt_nav2_planner/rrt_planner.hpp"

namespace rrt_nav2_planner
{

void RRTPlanner::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  std::string name,
  std::shared_ptr<tf2_ros::Buffer> tf,
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
  (void)tf;

  node_ = parent;
  name_ = name;
  costmap_ros_ = costmap_ros;
}

void RRTPlanner::cleanup()
{
  costmap_ros_.reset();
}

void RRTPlanner::activate()
{
}

void RRTPlanner::deactivate()
{
}

nav_msgs::msg::Path RRTPlanner::createPlan(
  const geometry_msgs::msg::PoseStamped & start,
  const geometry_msgs::msg::PoseStamped & goal)
{
  (void)start;
  (void)goal;

  nav_msgs::msg::Path path;

  return path;
}

}  // namespace rrt_nav2_planner