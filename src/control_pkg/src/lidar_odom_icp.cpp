#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_msgs/msg/tf_message.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "tf2/LinearMath/Matrix3x3.h"

#include "laser_geometry/laser_geometry.hpp"

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl_conversions/pcl_conversions.h>

#include <cmath>
#include <mutex>

using PointT = pcl::PointXYZ;
using CloudT = pcl::PointCloud<PointT>;

class LidarOdomICP : public rclcpp::Node
{
public:
    LidarOdomICP() : Node("lidar_odom_icp")
    {
        // ---- tunable params ----
        this->declare_parameter<double>("voxel_leaf_size", 0.03);      // m, downsample resolution
        this->declare_parameter<double>("icp_max_corr_dist", 0.03);     // m
        this->declare_parameter<int>("icp_max_iterations", 40);
        this->declare_parameter<double>("icp_transform_epsilon", 1e-8);
        this->declare_parameter<double>("icp_fitness_threshold", 0.5); // reject bad matches above this

        voxel_leaf_size_      = this->get_parameter("voxel_leaf_size").as_double();
        icp_max_corr_dist_    = this->get_parameter("icp_max_corr_dist").as_double();
        icp_max_iterations_   = this->get_parameter("icp_max_iterations").as_int();
        icp_transform_eps_    = this->get_parameter("icp_transform_epsilon").as_double();
        icp_fitness_thresh_   = this->get_parameter("icp_fitness_threshold").as_double();

        scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", rclcpp::SensorDataQoS(),
            std::bind(&LidarOdomICP::scan_callback, this, std::placeholders::_1));

        actual_info_sub_=this->create_subscription<tf2_msgs::msg::TFMessage>("/gt_tf", 10, std::bind(&LidarOdomICP::gt_callback, this, std::placeholders::_1));
        wheel_odom_sub=this->create_subscription<nav_msgs::msg::Odometry>("/wheel_odom", 10, std::bind(&LidarOdomICP::wheel_odom_callback, this, std::placeholders::_1));
        


        odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("/lidar_odom", 10);
    }

private:
    // running pose estimate (integrated from successive ICP transforms)
    double x_ = 0.0, y_ = 0.0, theta_ = 0.0;
    double gt_x_ = 0.0;
    double gt_y_ = 0.0;

    double prev_gt_x_ = 0.0;
    double prev_gt_y_ = 0.0;

    bool have_prev_gt_ = false;

    bool have_prev_cloud_ = false;
    CloudT::Ptr prev_cloud_;

    laser_geometry::LaserProjection projector_;

    double voxel_leaf_size_;
    double icp_max_corr_dist_;
    int    icp_max_iterations_;
    double icp_transform_eps_;
    double icp_fitness_thresh_;

    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
    rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr actual_info_sub_;


    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr wheel_odom_sub;
    double wheel_x_ = 0.0;
    double wheel_y_ = 0.0;
    double wheel_yaw_ = 0.0;

    double prev_wheel_x_ = 0.0;
    double prev_wheel_y_ = 0.0;
    double prev_wheel_yaw_ = 0.0;

    bool have_prev_wheel_pose_ = false;

    

    double normalizeAngle(double angle)
    {
        while (angle > M_PI) angle -= 2.0 * M_PI;
        while (angle < -M_PI) angle += 2.0 * M_PI;
        return angle;
    }

    // LaserScan -> PointCloud2 -> PCL cloud (flattened, z=0 since 2D lidar)
    CloudT::Ptr scanToCloud(const sensor_msgs::msg::LaserScan::SharedPtr &scan_msg)
    {
        sensor_msgs::msg::PointCloud2 cloud_msg;
        projector_.projectLaser(*scan_msg, cloud_msg);

        CloudT::Ptr cloud(new CloudT());
        pcl::fromROSMsg(cloud_msg, *cloud);

        // downsample so ICP isn't crunching every raw beam point
        CloudT::Ptr cloud_filtered(new CloudT());
        pcl::VoxelGrid<PointT> voxel;
        voxel.setInputCloud(cloud);
        voxel.setLeafSize(voxel_leaf_size_, voxel_leaf_size_, voxel_leaf_size_);
        voxel.filter(*cloud_filtered);

        return cloud_filtered;
    }

    void gt_callback(const tf2_msgs::msg::TFMessage::SharedPtr msg)
    {
        if (msg->transforms.empty())
            return;

        const auto &tf = msg->transforms[0];

        gt_x_ = tf.transform.translation.x;
        gt_y_ = tf.transform.translation.y;
    }

    void wheel_odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        wheel_x_ = msg->pose.pose.position.x;
        wheel_y_ = msg->pose.pose.position.y;

        tf2::Quaternion q(
            msg->pose.pose.orientation.x,
            msg->pose.pose.orientation.y,
            msg->pose.pose.orientation.z,
            msg->pose.pose.orientation.w);

        double roll, pitch, yaw;

        tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);

        wheel_yaw_ = yaw;
    }

    void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr scan_msg)
    {
        CloudT::Ptr current_cloud = scanToCloud(scan_msg);

        if (!have_prev_wheel_pose_)
        {
            prev_wheel_x_ = wheel_x_;
            prev_wheel_y_ = wheel_y_;
            prev_wheel_yaw_ = wheel_yaw_;

            have_prev_wheel_pose_ = true;

            prev_cloud_ = current_cloud;
            have_prev_cloud_ = true;

            return;
        }


        double wheel_dx = wheel_x_ - prev_wheel_x_;
        double wheel_dy = wheel_y_ - prev_wheel_y_;
        double wheel_dtheta = wheel_yaw_ - prev_wheel_yaw_;

        double cos_prev = std::cos(prev_wheel_yaw_);
        double sin_prev = std::sin(prev_wheel_yaw_);

        double local_dx =
            cos_prev * wheel_dx +
            sin_prev * wheel_dy;

        double local_dy =
        -sin_prev * wheel_dx +
            cos_prev * wheel_dy;

        Eigen::Matrix4f initial_guess = Eigen::Matrix4f::Identity();

        initial_guess(0, 0) = std::cos(wheel_dtheta);
        initial_guess(0, 1) = -std::sin(wheel_dtheta);

        initial_guess(1, 0) = std::sin(wheel_dtheta);
        initial_guess(1, 1) = std::cos(wheel_dtheta);

        initial_guess(0, 3) = local_dx;
        initial_guess(1, 3) = local_dy;

        prev_wheel_x_ = wheel_x_;
        prev_wheel_y_ = wheel_y_;
        prev_wheel_yaw_ = wheel_yaw_;


        printf(
            "Previous points: %zu | Current points: %zu\n",
            prev_cloud_ ? prev_cloud_->size() : 0,
            current_cloud->size()
        );

        double gt_dx = 0.0;
        double gt_dy = 0.0;

        if (have_prev_gt_)
        {
            gt_dx = gt_x_ - prev_gt_x_;
            gt_dy = gt_y_ - prev_gt_y_;
        }

        prev_gt_x_ = gt_x_;
        prev_gt_y_ = gt_y_;
        have_prev_gt_ = true;

        if (!have_prev_cloud_)
        {
            prev_cloud_ = current_cloud;
            have_prev_cloud_ = true;
            return;
        }

        if (current_cloud->empty() || prev_cloud_->empty())
        {
            RCLCPP_WARN(this->get_logger(), "Empty cloud, skipping ICP this cycle");
            prev_cloud_ = current_cloud;
            return;
        }

        // ---- ICP: align current scan onto previous scan ----
        pcl::IterativeClosestPoint<PointT, PointT> icp;
        icp.setInputSource(current_cloud);
        icp.setInputTarget(prev_cloud_);
        icp.setMaxCorrespondenceDistance(icp_max_corr_dist_);
        icp.setMaximumIterations(icp_max_iterations_);
        icp.setTransformationEpsilon(icp_transform_eps_);

        CloudT aligned;
        icp.align(aligned,initial_guess);

        if (!icp.hasConverged())
        {
            RCLCPP_WARN(this->get_logger(), "ICP did not converge, skipping this cycle");
            prev_cloud_ = current_cloud;
            return;
        }

        double fitness = icp.getFitnessScore();
        if (fitness > icp_fitness_thresh_)
        {
            RCLCPP_WARN(this->get_logger(),
                "ICP fitness score %.3f exceeds threshold %.3f, discarding match",
                fitness, icp_fitness_thresh_);
            prev_cloud_ = current_cloud;
            return;
        }

        // 4x4 homogeneous transform: prev_scan_frame <- current_scan_frame
        Eigen::Matrix4f T = icp.getFinalTransformation();
        printf(
            "ICP T: dx=%.4f dy=%.4f dtheta=%.3f deg fitness=%.4f\n",
            T(0,3),
            T(1,3),
            std::atan2(T(1,0), T(0,0)) * 180.0 / M_PI,
            fitness
        );

        double dx = T(0, 3);
        double dy = T(1, 3);
        double dtheta = std::atan2(T(1, 0), T(0, 0)); // yaw from rotation block

        printf(
            "GT movement: dx=%.4f dy=%.4f | ICP: dx=%.4f dy=%.4f\n",
            gt_dx, gt_dy, dx, dy
        );

        // ---- integrate scan-to-scan delta into running world-frame pose ----
        // rotate the local (dx, dy) delta into the current global heading before adding
        double cos_t = std::cos(theta_);
        double sin_t = std::sin(theta_);

        x_ += dx * cos_t - dy * sin_t;
        y_ += dx * sin_t + dy * cos_t;
        theta_ = normalizeAngle(theta_ + dtheta);

        prev_cloud_ = current_cloud;

        publishOdom(scan_msg->header.stamp);
    }

    void publishOdom(const rclcpp::Time &stamp)
    {
        nav_msgs::msg::Odometry odom;
        odom.header.stamp = stamp;
        odom.header.frame_id = "odom";
        odom.child_frame_id = "base_link";

        odom.pose.pose.position.x = x_;
        odom.pose.pose.position.y = y_;
        odom.pose.pose.position.z = 0.0;

        tf2::Quaternion q;
        q.setRPY(0, 0, theta_);
        odom.pose.pose.orientation.x = q.x();
        odom.pose.pose.orientation.y = q.y();
        odom.pose.pose.orientation.z = q.z();
        odom.pose.pose.orientation.w = q.w();

        // ICP-based odom drifts differently than wheel odom — give the EKF
        // reasonable starting covariances so it doesn't over-trust this source.
        for (auto &c : odom.pose.covariance) c = 0.0;
        odom.pose.covariance[0]  = 0.02; // x
        odom.pose.covariance[7]  = 0.02; // y
        odom.pose.covariance[35] = 0.05; // yaw

        odom_pub_->publish(odom);
    }




};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<LidarOdomICP>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}