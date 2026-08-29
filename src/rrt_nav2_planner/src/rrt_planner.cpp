#include "rrt_nav2_planner/rrt_planner.hpp"
#include <iostream>
#include <vector>
#include <cmath>
#include <random>
#include <tuple>
#include <algorithm>
#include <limits>

namespace rrt_nav2_planner
{

// A simple 2D point (x, y)
struct Point {
    double x;
    double y;
};

// An RRT tree node: stores the parent coordinates and this node's coordinates
struct TreeNode {
    double parentX;
    double parentY;
    double x;
    double y;
};

// Axis-aligned rectangle obstacle (xmin, xmax, ymin, ymax)
struct Obstacle {
    double xmin;
    double xmax;
    double ymin;
    double ymax;
};


std::vector<Obstacle> obstacles = {
    {4, 6, 3, 6},
    {7, 8.5, 7, 9},
    {1, 2, 6, 8}
};


// random number generation, replacing python's `random` module
std::mt19937 rng(std::random_device{}());
std::uniform_real_distribution<double> uniform01(0.0, 1.0);
std::uniform_real_distribution<double> uniform0to10(0.0, 10.0);

// ---------------------------------------------------------------------------
// round a double to 2 decimal places (mirrors python's round(x, 2))
// ---------------------------------------------------------------------------
double round2(double v) {
    return std::round(v * 100.0) / 100.0;
}

// ---------------------------------------------------------------------------
// distance between two points (euclidean distance, using hypot)
// ---------------------------------------------------------------------------
double distanceFn(double x1, double y1, double x2, double y2) {
    return std::hypot(x2 - x1, y2 - y1);
}

double nearestPoint(double x, double y, double xGoal, double yGoal) {
    return std::sqrt((xGoal - x) * (xGoal - x) + (yGoal - y) * (yGoal - y));
}

// ---------------------------------------------------------------------------
// generate a random point in [0,10] x [0,10], rounded to 2 decimals
//
// NOTE: this range is a leftover from the original standalone script's fixed
// 10x10 test world. For real use against a costmap, this should sample
// within the costmap's actual bounds instead - flagged here since it's easy
// to miss (RRT will never find anything outside the [0,10]x[0,10] box).
// ---------------------------------------------------------------------------
Point randomPointGenerator() {
    double x = round2(uniform0to10(rng));
    double y = round2(uniform0to10(rng));
    return {x, y};
}

// forward declaration
bool collisionFree(double x, double y, double xGoal, double yGoal,
                    const std::vector<Obstacle>& obstacles);

// ---------------------------------------------------------------------------
// check if the straight line between two points collides with any obstacle
// rectangle.
// ---------------------------------------------------------------------------
bool collisionFree(double x, double y,
                    double xGoal, double yGoal,
                    const std::vector<Obstacle>& obstacles)
{
    for (int i = 0; i <= 100; i++) {
        double t = i / 100.0;
        double lineX = x + t * (xGoal - x);
        double lineY = y + t * (yGoal - y);

        for (const auto& ob : obstacles) {
            double xmin = ob.xmin, xmax = ob.xmax, ymin = ob.ymin, ymax = ob.ymax;
            if (xmin <= lineX && lineX <= xmax && ymin <= lineY && lineY <= ymax) {
                return false;
            }
        }
    }
    return true;
}

// ---------------------------------------------------------------------------
// check whether a single point lies inside any obstacle rectangle
// FIXED: takes obstacles by const reference (was a non-const `&`, which
// would refuse to bind to the const vectors passed everywhere else).
// ---------------------------------------------------------------------------


bool RRTPlanner::pointCollision(double x, double y) {
    unsigned int mx;  //world cells
    unsigned int my;

    if (!costmap_ros_->getCostmap()->worldToMap(x,y,mx,my)){    //costmap_ros_->getCostmap() gives actual Costmap2d, .worldToMao(x,y,mx,my) convert RRT world corrdinates into a costmap cell, takes x,y and fills mx,my
        return true;  //if (x,y) cannot be converted to valid costmap cell then return true
    }

    //get cost of the cell
    unsigned char cost = costmap_ros_->getCostmap()->getCost(mx, my);


    if (cost>=nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE){     //inscribed.... has cost 253 ,so we reject cells with cost 253, ane 254
        return true;
    }
    return false;
}

// ---------------------------------------------------------------------------
// if the target is further than step_size away, "steer" towards it by
// step_size; otherwise go straight to it. Returns (x, y, isCollisionFree)
// FIXED: both collisionFree() calls now consistently use the `obstacles`
// parameter (one previously referenced a nonexistent `obstacle`/`obstacles`
// typo depending on version).
// ---------------------------------------------------------------------------
std::tuple<double, double, bool> steer(double x, double y, double xGoal, double yGoal,
                                        double stepSize, double bestDistance,
                                        const std::vector<Obstacle>& obstacles) {
    if (bestDistance <= stepSize) {
        bool collision = collisionFree(x, y, xGoal, yGoal, obstacles);
        return {xGoal, yGoal, collision};
    } else {
        double dx = xGoal - x;
        double dy = yGoal - y;
        double length = std::hypot(dx, dy);

        double unitX = dx / length;
        double unitY = dy / length;

        double newX = x + unitX * stepSize;
        double newY = y + unitY * stepSize;

        bool collision = collisionFree(x, y, newX, newY, obstacles);
        return {newX, newY, collision};
    }
}

// ---------------------------------------------------------------------------
// Bezier / corner-rounding smoothing helpers
// ---------------------------------------------------------------------------

std::pair<Point, Point> perpendicularPoints(double Ax, double Ay, double Bx, double By,
                                             double Cx, double Cy, double distanceOffset = 0.5) {
    double dx = Cx - Ax;
    double dy = Cy - Ay;

    double perpX = -dy;
    double perpY = dx;

    double length = std::hypot(perpX, perpY);

    double unitPerpX = perpX / length;
    double unitPerpY = perpY / length;

    Point pPlus  = {Bx + unitPerpX * distanceOffset, By + unitPerpY * distanceOffset};
    Point pMinus = {Bx - unitPerpX * distanceOffset, By - unitPerpY * distanceOffset};

    return {pPlus, pMinus};
}

Point bezier(const Point& P1, const Point& B, const Point& P2, double t) {
    double x = (1 - t) * (1 - t) * P1.x + 2 * (1 - t) * t * B.x + t * t * P2.x;
    double y = (1 - t) * (1 - t) * P1.y + 2 * (1 - t) * t * B.y + t * t * P2.y;
    return {x, y};
}

std::vector<Point> findSmoothPath(const Point& A, const Point& B, const Point& C,
                                   double rounding = 0.2, int curvePoints = 30) {
    (void)rounding;
    std::vector<Point> smoothPathPts;

    Point startCurve = A;
    Point control = B;
    Point endCurve = C;

    smoothPathPts.push_back(startCurve);

    for (int j = 1; j <= curvePoints; j++) {
        double t = static_cast<double>(j) / curvePoints;
        Point point = bezier(startCurve, control, endCurve, t);
        smoothPathPts.push_back(point);
    }

    return smoothPathPts;
}

// FIXED: now takes `obstacles` as a parameter (was reading a nonexistent
// global) and forwards it to collisionFree().
bool checkIfSmoothPathCollide(const std::vector<Point>& smoothPathPts,
                               const std::vector<Obstacle>& obstacles) {
    for (size_t i = 0; i + 1 < smoothPathPts.size(); i++) {
        bool isCollisionFree = collisionFree(smoothPathPts[i].x, smoothPathPts[i].y,
                                              smoothPathPts[i + 1].x, smoothPathPts[i + 1].y,
                                              obstacles);
        if (!isCollisionFree) {
            std::cout << "not collision free" << std::endl;
            return true;
        }
    }
    return false;
}

// FIXED: now takes `obstacles` as a parameter and forwards it to every
// pointCollision()/checkIfSmoothPathCollide() call inside (there were
// several - one per candidate curve: sPath, pPlusPath, pMinusPath).
std::vector<Point> smoothPathBezier(const std::vector<Point>& path,
                                     const std::vector<Obstacle>& obstacles,
                                     double rounding = 1.0, int curvePoints = 30) {
    if (path.size() < 3) {
        return path;
    }

    std::vector<Point> smoothPathFinal;
    smoothPathFinal.push_back(path[0]);

    for (size_t i = 1; i + 1 < path.size(); i++) {
        Point A = path[i - 1];
        Point B = path[i];
        Point C = path[i + 1];

        Point P1 = {
            B.x + rounding * (A.x - B.x),
            B.y + rounding * (A.y - B.y)
        };

        Point P2 = {
            B.x + rounding * (C.x - B.x),
            B.y + rounding * (C.y - B.y)
        };

        auto pPoints = perpendicularPoints(P1.x, P1.y, B.x, B.y, P2.x, P2.y);
        Point pPlus = pPoints.first;
        Point pMinus = pPoints.second;

        bool pPlusInObstacle = pointCollision(pPlus.x, pPlus.y, obstacles);
        bool pMinusInObstacle = pointCollision(pMinus.x, pMinus.y, obstacles);
        (void)pPlusInObstacle;
        (void)pMinusInObstacle;

        std::vector<Point> sPath = findSmoothPath(P1, B, P2);
        bool checkPathCollision = checkIfSmoothPathCollide(sPath, obstacles);
        std::cout << "do b collide " << (checkPathCollision ? "true" : "false") << std::endl;

        if (checkPathCollision) {
            std::vector<Point> pPlusPath = findSmoothPath(P1, pPlus, P2);
            bool checkPPlusCollision = checkIfSmoothPathCollide(pPlusPath, obstacles);
            std::cout << "do pplus collide: " << (checkPPlusCollision ? "true" : "false") << std::endl;

            if (checkPPlusCollision) {
                std::vector<Point> pMinusPath = findSmoothPath(P1, pMinus, P2);
                bool checkPMinusCollision = checkIfSmoothPathCollide(pMinusPath, obstacles);
                if (checkPMinusCollision) {
                    std::cout << "p minus collide" << std::endl;
                } else {
                    std::cout << "p_minus is correct" << std::endl;
                    smoothPathFinal.insert(smoothPathFinal.end(), pMinusPath.begin(), pMinusPath.end());
                }
            } else {
                std::cout << "p_plus is correct" << std::endl;
                smoothPathFinal.insert(smoothPathFinal.end(), pPlusPath.begin(), pPlusPath.end());
            }
        } else {
            std::cout << "b is correct" << std::endl;
            smoothPathFinal.insert(smoothPathFinal.end(), sPath.begin(), sPath.end());
        }
    }

    smoothPathFinal.push_back(path.back());
    return smoothPathFinal;
}

// ---------------------------------------------------------------------------
// PCHIP interpolation (Fritsch-Carlson monotone cubic Hermite method,
// matching scipy.interpolate.PchipInterpolator)
// ---------------------------------------------------------------------------

std::vector<double> pchipSlopes(const std::vector<double>& t, const std::vector<double>& y) {
    int n = static_cast<int>(t.size());
    std::vector<double> h(n - 1), delta(n - 1);

    for (int i = 0; i < n - 1; i++) {
        h[i] = t[i + 1] - t[i];
        delta[i] = (y[i + 1] - y[i]) / h[i];
    }

    std::vector<double> d(n, 0.0);

    if (n == 2) {
        d[0] = d[1] = delta[0];
        return d;
    }

    for (int i = 1; i < n - 1; i++) {
        if (delta[i - 1] * delta[i] <= 0.0) {
            d[i] = 0.0;
        } else {
            double w1 = 2 * h[i] + h[i - 1];
            double w2 = h[i] + 2 * h[i - 1];
            d[i] = (w1 + w2) / (w1 / delta[i - 1] + w2 / delta[i]);
        }
    }

    auto endpointDerivative = [](double h0, double h1, double delta0, double delta1) {
        double d0 = ((2 * h0 + h1) * delta0 - h0 * delta1) / (h0 + h1);
        if (d0 * delta0 <= 0.0) {
            d0 = 0.0;
        } else if ((delta0 * delta1 <= 0.0) && (std::fabs(d0) > std::fabs(3 * delta0))) {
            d0 = 3 * delta0;
        }
        return d0;
    };

    d[0] = endpointDerivative(h[0], h[1], delta[0], delta[1]);
    d[n - 1] = endpointDerivative(h[n - 2], h[n - 3], delta[n - 2], delta[n - 3]);

    return d;
}

double hermiteEval(double t0, double t1, double y0, double y1, double m0, double m1, double tt) {
    double h = t1 - t0;
    double s = (tt - t0) / h;
    double h00 = 2 * s * s * s - 3 * s * s + 1;
    double h10 = s * s * s - 2 * s * s + s;
    double h01 = -2 * s * s * s + 3 * s * s;
    double h11 = s * s * s - s * s;
    return h00 * y0 + h10 * h * m0 + h01 * y1 + h11 * h * m1;
}

std::vector<double> pchipInterpolate(const std::vector<double>& t, const std::vector<double>& y,
                                      const std::vector<double>& tQuery) {
    std::vector<double> slopes = pchipSlopes(t, y);
    std::vector<double> result;
    result.reserve(tQuery.size());

    for (double tq : tQuery) {
        int i = 0;
        while (i < static_cast<int>(t.size()) - 2 && tq > t[i + 1]) {
            i++;
        }
        result.push_back(hermiteEval(t[i], t[i + 1], y[i], y[i + 1], slopes[i], slopes[i + 1], tq));
    }

    return result;
}

// ===========================================================================
// planRRT() - THE RRT PLANNER
// Input:  start, goal, obstacles (+ tuning params with defaults)
// Output: shortcut waypoint path from start to goal
// ===========================================================================
std::vector<Point> planRRT(const Point& start, const Point& goal,
                            const std::vector<Obstacle>& obstacles,
                            int maxIterations = 500, double stepSize = 1.0,
                            double goalTolerance = 0.2) {
    std::vector<TreeNode> tree = { {start.x, start.y, start.x, start.y} };

    int i = 0;
    while (i < maxIterations) {
        Point target;
        if (uniform01(rng) <= 0.1) {
            target = goal;
        } else {
            target = randomPointGenerator();
        }

        double bestDistance = std::numeric_limits<double>::infinity();
        Point nearestPointP{0, 0};
        double targetX = target.x;
        double targetY = target.y;

        for (const auto& node : tree) {
            double x = node.x;
            double y = node.y;
            double d = nearestPoint(x, y, targetX, targetY);
            if (d < bestDistance) {
                bestDistance = d;
                nearestPointP = {x, y};
            }
        }

        double nearestPointPX = nearestPointP.x;
        double nearestPointPY = nearestPointP.y;

        auto [x, y, collision] = steer(nearestPointPX, nearestPointPY, targetX, targetY,
                                        stepSize, bestDistance, obstacles);

        if (collision) {
            tree.push_back({nearestPointPX, nearestPointPY, x, y});

            if (distanceFn(x, y, goal.x, goal.y) < goalTolerance) {
                std::cout << "goal reached" << std::endl;
                break;
            }
        }

        std::cout << " Target:(" << targetX << "," << targetY << ")"
                  << " Nearest point:(" << nearestPointP.x << "," << nearestPointP.y << ")"
                  << " Distance:" << bestDistance
                  << " Selected point:(" << x << "," << y << ")"
                  << " Collision:" << (collision ? "true" : "false")
                  << " len:" << tree.size() << std::endl;

        i++;
    }

    double treeLastX = tree.back().x;
    double treeLastY = tree.back().y;
    std::vector<Point> path = { {treeLastX, treeLastY} };

    while (!(treeLastX == tree[0].x && treeLastY == tree[0].y)) {
        for (const auto& node : tree) {
            if (node.x == treeLastX && node.y == treeLastY) {
                double parentX = node.parentX;
                double parentY = node.parentY;
                path.push_back({parentX, parentY});
                treeLastX = parentX;
                treeLastY = parentY;
                break;
            }
        }
    }

    std::reverse(path.begin(), path.end());

    std::vector<Point> newPath;
    double mainX = path[0].x;
    double mainY = path[0].y;

    while (true) {
        if (!(mainX == path.back().x && mainY == path.back().y)) {
            newPath.push_back({mainX, mainY});
        } else {
            newPath.push_back({mainX, mainY});
            break;
        }

        for (int j = static_cast<int>(path.size()) - 1; j >= 0; j--) {
            double checkX = path[j].x;
            double checkY = path[j].y;
            bool checkCollisionFree = collisionFree(mainX, mainY, checkX, checkY, obstacles);
            if (checkCollisionFree) {
                mainX = checkX;
                mainY = checkY;
                break;
            }
        }
    }

    return newPath;
}

// ===========================================================================
// smoothPathPCHIP() - final curve smoothing pass
// ===========================================================================
std::vector<Point> smoothPathPCHIP(const std::vector<Point>& path, int numFine = 200) {
    std::vector<double> xVals, yVals, tVals;
    for (size_t idx = 0; idx < path.size(); idx++) {
        xVals.push_back(path[idx].x);
        yVals.push_back(path[idx].y);
        tVals.push_back(static_cast<double>(idx));
    }

    std::vector<double> tFine;
    for (int k = 0; k < numFine; k++) {
        tFine.push_back(static_cast<double>(k) * (path.size() - 1) / (numFine - 1));
    }

    std::vector<double> xFine = pchipInterpolate(tVals, xVals, tFine);
    std::vector<double> yFine = pchipInterpolate(tVals, yVals, tFine);

    std::vector<Point> result;
    for (int k = 0; k < numFine; k++) {
        result.push_back({xFine[k], yFine[k]});
    }
    return result;
}

// ===========================================================================
// buildObstaclesFromCostmap() - NEW.
// Converts occupied cells in the live Nav2 costmap into the Obstacle
// rectangles that collisionFree()/pointCollision() expect, so planRRT() can
// be reused unchanged against real sensor data instead of a hardcoded list.
//
// NOTE: this uses the standard nav2_costmap_2d::Costmap2D API
// (getSizeInCellsX/Y, getCost, mapToWorld, LETHAL_OBSTACLE/INSCRIBED_INFLATED
// constants). It has NOT been compiled against real Nav2 headers in this
// environment (ROS 2/Nav2 isn't installed here) - double check it builds
// against your actual nav2_costmap_2d version, and treat this function as a
// starting point rather than a verified drop-in.
//
// This emits one small rectangle per occupied cell, which is correct but not
// efficient for a dense costmap - if collisionFree() ends up too slow, the
// better long-term fix is to have collisionFree()/pointCollision() query
// costmap_ros_->getCostmap()->getCost(...) directly instead of building an
// Obstacle list at all.
// ===========================================================================


// ---------------------------------------------------------------------------
// RRTPlanner lifecycle methods
// ---------------------------------------------------------------------------

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

// FIXED:
//  - built `obstacles` from the live costmap instead of referencing an
//    undefined variable
//  - removed the duplicate `nav_msgs::msg::Path path;` declaration
//  - completed the per-waypoint loop: sets a valid orientation, stamps each
//    pose, and actually pushes it into path.poses (the original loop built
//    a PoseStamped and then discarded it)
nav_msgs::msg::Path RRTPlanner::createPlan(
  const geometry_msgs::msg::PoseStamped & start,
  const geometry_msgs::msg::PoseStamped & goal,
  std::function<bool()> cancel_checker)
{
  (void)cancel_checker;

  Point rrtStart = {
    start.pose.position.x,
    start.pose.position.y
  };

  Point rrtGoal = {
    goal.pose.position.x,
    goal.pose.position.y
  };

//   std::vector<Obstacle> obstacles = buildObstaclesFromCostmap(costmap_ros_);

  std::vector<Point> rrtPath = planRRT(rrtStart, rrtGoal, obstacles);

  nav_msgs::msg::Path path; //path contains sequence of posestamped
  path.header = start.header;

  //convert rrt path into nav2 path message
  for (const auto & point : rrtPath) {   //loop through each rrt point and create posestamped
    geometry_msgs::msg::PoseStamped pose;
    pose.header = start.header;
    pose.pose.position.x = point.x;
    pose.pose.position.y = point.y;
    pose.pose.position.z = 0.0;
    pose.pose.orientation.w = 1.0;   // identity orientation - no heading info from RRT itself
    path.poses.push_back(pose);
  }

  return path;
}

}  // namespace rrt_nav2_planner