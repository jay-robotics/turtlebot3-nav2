#ifndef RRT_NAV2_PLANNER__RRT_PLANNER_HPP_
#define RRT_NAV2_PLANNER__RRT_PLANNER_HPP_

#include <memory>
#include <string>

#include "nav2_core/global_planner.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include <functional>

namespace rrt_nav2_planner
{

class RRTPlanner : public nav2_core::GlobalPlanner
{
public:

  void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    std::string name,
    std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;

  void cleanup() override;

  void activate() override;

  void deactivate() override;

  nav_msgs::msg::Path createPlan(
    const geometry_msgs::msg::PoseStamped & start,
    const geometry_msgs::msg::PoseStamped & goal,
    std::function<bool()> cancel_checker) override;

private:

  rclcpp_lifecycle::LifecycleNode::WeakPtr node_;
  std::string name_;

  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;

};

}  // namespace rrt_nav2_planner

#endif  // RRT_NAV2_PLANNER__RRT_PLANNER_HPP_