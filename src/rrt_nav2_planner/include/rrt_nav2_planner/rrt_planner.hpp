#ifndef RRT_NAV2_PLANNER__RRT_PLANNER_HPP_
#define RRT_NAV2_PLANNER__RRT_PLANNER_HPP_

#include <memory>
#include <string>
#include <vector>
#include <tuple>

#include "nav2_core/global_planner.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include <functional>

namespace rrt_nav2_planner
{

struct Point{
  double x;
  double y;
};


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

  std::vector<Point> planRRT(const Point& start, const Point& goal,
                            int maxIterations = 500, double stepSize = 1.0,
                            double goalTolerance = 0.2);

private:

  bool pointCollision(double x,double y);

  bool collisionFree(
    double x,
    double y,
    double xGoal,
    double yGoal);

  std::tuple<double, double, bool>steer(

    double x, double y,
    double xGoal, double yGoal,
    double stepSize, double bestDistance
  );

  bool checkIfSmoothPathCollide( const std::vector<Point>& smoothPathPts);

  std::vector<Point> smoothPathBezier( const std::vector<Point>& path,
                                        double rounding = 1.0, int curvePoints = 30);

  rclcpp_lifecycle::LifecycleNode::WeakPtr node_;
  std::string name_;

  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;

};

}  // namespace rrt_nav2_planner

#endif  // RRT_NAV2_PLANNER__RRT_PLANNER_HPP_