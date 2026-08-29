/*
 * RRT (Rapidly-exploring Random Tree) Path Planner - C++ port
 * ------------------------------------------------------------
 * Direct translation of the original Python RRT script, now refactored so
 * that the RRT search itself lives in one function (planRRT) and the final
 * curve smoothing lives in another (smoothPathPCHIP), instead of both being
 * written directly in main(). main() now just calls the two in sequence.
 * This mirrors the shape createPlan() will eventually need to have in a
 * Nav2 GlobalPlanner plugin.
 *
 * CHANGES vs. the previous version (flagged explicitly, not silent):
 *   1. planRRT() and smoothPathPCHIP() are new - they contain code that used
 *      to sit directly in main().
 *   2. `best_distance` is no longer a global variable read implicitly by
 *      steer(). It is now passed into steer() as an explicit parameter.
 *      This was a bug-ish quirk in the original Python (and in the first
 *      C++ port). It had to be fixed here because planRRT() needs to be a
 *      self-contained, reusable function - relying on a mutable global for
 *      its internal state would break the moment it's called more than
 *      once (which Nav2 will do, once per planning request).
 *   3. `tree` and `goal` are no longer globals - `tree` is now local to
 *      planRRT(), and `goal`/`start` are passed in as parameters.
 *   4. The leftover unused `start` vector at the bottom of the original
 *      main() (dead code that only fed the removed plotting) is renamed to
 *      `legacyUnusedPoints` to avoid clashing with the new `start` Point
 *      parameter used for planning.
 * No other logic was changed - collision checking, steering, backtracking,
 * shortcutting, and the PCHIP math are identical to before.
 *
 * Compile with:  g++ -std=c++17 -O2 rrt_planner_refactored.cpp -o rrt_planner
 */

#include <iostream>
#include <vector>
#include <cmath>
#include <random>
#include <tuple>
#include <algorithm>
#include <limits>

// ---------------------------------------------------------------------------
// Data structures
// ---------------------------------------------------------------------------

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

// ---------------------------------------------------------------------------
// Global state
// ---------------------------------------------------------------------------
// NOTE: `obstacles` stays global for now (this is what your world/costmap
// data would replace later in Nav2). `tree`, `goal`, and `best_distance`
// used to be globals too, but have been moved to be local to planRRT()
// (see the CHANGES note above).
// ---------------------------------------------------------------------------

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

// distance between two points - kept as a separate function to mirror the
// original python code's near-duplicate helper.
double nearestPoint(double x, double y, double xGoal, double yGoal) {
    return std::sqrt((xGoal - x) * (xGoal - x) + (yGoal - y) * (yGoal - y));
}

// ---------------------------------------------------------------------------
// generate a random point in [0,10] x [0,10], rounded to 2 decimals
// ---------------------------------------------------------------------------
Point randomPointGenerator() {
    double x = round2(uniform0to10(rng));
    double y = round2(uniform0to10(rng));
    return {x, y};
}

// forward declaration (steer() calls collisionFree())
bool collisionFree(double x, double y, double xGoal, double yGoal,
                    double xminDefault = 4, double xmaxDefault = 6,
                    double yminDefault = 3, double ymaxDefault = 7);

// ---------------------------------------------------------------------------
// check if the straight line between two points collides with any obstacle
// rectangle. The default parameters are dead code (shadowed by the for-loop
// over the global `obstacles` list) - same as the original python function.
// ---------------------------------------------------------------------------
bool collisionFree(double x, double y, double xGoal, double yGoal,
                    double xminDefault, double xmaxDefault,
                    double yminDefault, double ymaxDefault) {
    (void)xminDefault; (void)xmaxDefault; (void)yminDefault; (void)ymaxDefault;

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
// ---------------------------------------------------------------------------
bool pointCollision(double x, double y) {
    for (const auto& ob : obstacles) {
        if (ob.xmin <= x && x <= ob.xmax && ob.ymin <= y && y <= ob.ymax) {
            return true;
        }
    }
    return false;
}

// ---------------------------------------------------------------------------
// if the target is further than step_size away, "steer" towards it by
// step_size; otherwise go straight to it. Returns (x, y, isCollisionFree)
//
// CHANGED: bestDistance is now an explicit parameter instead of being read
// from a global. Same math as before, just no longer relying on outer scope.
// ---------------------------------------------------------------------------
std::tuple<double, double, bool> steer(double x, double y, double xGoal, double yGoal,
                                        double stepSize, double bestDistance) {
    if (bestDistance <= stepSize) {
        bool collision = collisionFree(x, y, xGoal, yGoal);
        return {xGoal, yGoal, collision};
    } else {
        double dx = xGoal - x;
        double dy = yGoal - y;
        double length = std::hypot(dx, dy);

        double unitX = dx / length;
        double unitY = dy / length;

        double newX = x + unitX * stepSize;
        double newY = y + unitY * stepSize;

        bool collision = collisionFree(x, y, newX, newY);
        return {newX, newY, collision};
    }
}

// ---------------------------------------------------------------------------
// Bezier / corner-rounding smoothing helpers (unchanged from before)
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

bool checkIfSmoothPathCollide(const std::vector<Point>& smoothPathPts) {
    for (size_t i = 0; i + 1 < smoothPathPts.size(); i++) {
        bool isCollisionFree = collisionFree(smoothPathPts[i].x, smoothPathPts[i].y,
                                              smoothPathPts[i + 1].x, smoothPathPts[i + 1].y);
        if (!isCollisionFree) {
            std::cout << "not collision free" << std::endl;
            return true;
        }
    }
    return false;
}

std::vector<Point> smoothPathBezier(const std::vector<Point>& path, double rounding = 1.0, int curvePoints = 30) {
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

        bool pPlusInObstacle = pointCollision(pPlus.x, pPlus.y);
        bool pMinusInObstacle = pointCollision(pMinus.x, pMinus.y);
        (void)pPlusInObstacle;
        (void)pMinusInObstacle;

        std::vector<Point> sPath = findSmoothPath(P1, B, P2);
        bool checkPathCollision = checkIfSmoothPathCollide(sPath);
        std::cout << "do b collide " << (checkPathCollision ? "true" : "false") << std::endl;

        if (checkPathCollision) {
            std::vector<Point> pPlusPath = findSmoothPath(P1, pPlus, P2);
            bool checkPPlusCollision = checkIfSmoothPathCollide(pPlusPath);
            std::cout << "do pplus collide: " << (checkPPlusCollision ? "true" : "false") << std::endl;

            if (checkPPlusCollision) {
                std::vector<Point> pMinusPath = findSmoothPath(P1, pMinus, P2);
                bool checkPMinusCollision = checkIfSmoothPathCollide(pMinusPath);
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
// matching scipy.interpolate.PchipInterpolator - see earlier verification)
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
// planRRT() - THE RRT PLANNER, now a standalone function.
//
// Input:
//   start          - starting point of the search
//   goal           - target point to reach
//   maxIterations  - how many sampling iterations to attempt (default 500)
//   stepSize       - how far each steer() step advances toward the target
//   goalTolerance  - how close a new node must get to `goal` to stop early
//
// Output:
//   A std::vector<Point> - the SHORTCUT path from start to goal (i.e. the
//   raw tree-search path with redundant waypoints removed by the greedy
//   collision-free shortcutting pass). This is the path that gets handed
//   to smoothPathPCHIP() afterwards. If no path is found within
//   maxIterations, this returns whatever path leads to the last tree node
//   added (may not reach the goal).
// ===========================================================================
std::vector<Point> planRRT(const Point& start, const Point& goal,
                            int maxIterations = 500, double stepSize = 1.0,
                            double goalTolerance = 0.2) {
    // tree is now local to this function (was global before)
    std::vector<TreeNode> tree = { {start.x, start.y, start.x, start.y} };

    int i = 0;
    while (i < maxIterations) {
        Point target;
        // 10% of the time, sample the goal directly (goal biasing)
        if (uniform01(rng) <= 0.1) {
            target = goal;
        } else {
            target = randomPointGenerator();
        }

        // find the point already in the tree that is nearest to the target
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

        // NOTE: bestDistance is now passed explicitly (see steer() above)
        auto [x, y, collision] = steer(nearestPointPX, nearestPointPY, targetX, targetY,
                                        stepSize, bestDistance);

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

    // -----------------------------------------------------------------
    // Backtrack from the last added tree node to the start, building `path`
    // -----------------------------------------------------------------
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

    // -----------------------------------------------------------------
    // Shortcut the path: greedily jump to the furthest waypoint reachable
    // by a straight, collision-free line.
    // -----------------------------------------------------------------
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
            bool checkCollisionFree = collisionFree(mainX, mainY, checkX, checkY);
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
// smoothPathPCHIP() - THE FINAL CURVE SMOOTHING PASS.
//
// Input:
//   path - a std::vector<Point> waypoint path (typically planRRT()'s output)
//
// Output:
//   A std::vector<Point> - a densely-sampled (200 points), smooth curve
//   running through the same waypoints, computed via monotone cubic
//   Hermite (PCHIP) interpolation over both x(t) and y(t).
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

// ---------------------------------------------------------------------------
// main - now just calls planRRT() then smoothPathPCHIP()
// ---------------------------------------------------------------------------
int main() {
    Point start = {1, 1};
    Point goal = {9, 9};

    // 1) Run the RRT search + shortcutting -> raw/shortcut waypoint path
    std::vector<Point> newPath = planRRT(start, goal);

    std::cout << std::endl << "new path: ";
    for (const auto& p : newPath) {
        std::cout << "(" << p.x << "," << p.y << ") ";
    }
    std::cout << std::endl;

    // Corner-rounding / bezier smoothing pass - computed for parity with the
    // original script, though its result isn't the one used for the final
    // printed/checked curve below (same as before the refactor).
    std::vector<Point> smoothPathFinalBezier = smoothPathBezier(newPath, 0.5, 30);
    (void)smoothPathFinalBezier;

    // 2) Run the PCHIP smoothing pass -> final smooth curve
    std::vector<Point> smoothPathFinal = smoothPathPCHIP(newPath);

    bool smoothPathHasCollision = checkIfSmoothPathCollide(smoothPathFinal);
    if (smoothPathHasCollision) {
        std::cout << "smooth_path_final is NOT collision free" << std::endl;
    } else {
        std::cout << "smooth_path_final is collision free" << std::endl;
    }

    // Leftover unused sample data from the original script - renamed from
    // `start` to `legacyUnusedPoints` to avoid clashing with the `start`
    // Point above. Only fed the (now removed) matplotlib plotting.
    std::vector<Point> legacyUnusedPoints = { {1, 1}, {2, 2}, {3, 2}, {4, 3} };
    (void)legacyUnusedPoints;

    double d = distanceFn(2, 3, 7, 6);
    (void)d;

    return 0;
}