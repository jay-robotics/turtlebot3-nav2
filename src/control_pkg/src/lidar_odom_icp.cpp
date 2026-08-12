#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "tf2/LinearMath/Quaternion.h"

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
        this->declare_parameter<double>("voxel_leaf_size", 0.03);
        this->declare_parameter<double>("icp_max_corr_dist", 0.3);
        this->declare_parameter<int>("icp_max_iterations", 40);
        this->declare_parameter<double>("icp_transform_epsilon", 1e-8);
        this->declare_parameter<double>("icp_fitness_threshold", 0.5);

        voxel_leaf_size_      = this->get_parameter("voxel_leaf_size").as_double();
        icp_max_corr_dist_    = this->get_parameter("icp_max_corr_dist").as_double();
        icp_max_iterations_   = this->get_parameter("icp_max_iterations").as_int();
        icp_transform_eps_    = this->get_parameter("icp_transform_epsilon").as_double();
        icp_fitness_thresh_   = this->get_parameter("icp_fitness_threshold").as_double();

        scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", rclcpp::SensorDataQoS(),
            std::bind(&LidarOdomICP::scan_callback, this, std::placeholders::_1));

        odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("/lidar_odom", 10);
    }

private:
    double x_ = 0.0, y_ = 0.0, theta_ = 0.0;

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

    double normalizeAngle(double angle)
    {
        while (angle > M_PI) angle -= 2.0 * M_PI;
        while (angle < -M_PI) angle += 2.0 * M_PI;
        return angle;
    }

    CloudT::Ptr scanToCloud(const sensor_msgs::msg::LaserScan::SharedPtr &scan_msg)
    {
        sensor_msgs::msg::PointCloud2 cloud_msg;
        projector_.projectLaser(*scan_msg, cloud_msg);

        CloudT::Ptr cloud(new CloudT());
        pcl::fromROSMsg(cloud_msg, *cloud);

        CloudT::Ptr cloud_filtered(new CloudT());
        pcl::VoxelGrid<PointT> voxel;
        voxel.setInputCloud(cloud);
        voxel.setLeafSize(voxel_leaf_size_, voxel_leaf_size_, voxel_leaf_size_);
        voxel.filter(*cloud_filtered);

        return cloud_filtered;
    }

    void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr scan_msg)
    {
        CloudT::Ptr current_cloud = scanToCloud(scan_msg);

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

        pcl::IterativeClosestPoint<PointT, PointT> icp;
        icp.setInputSource(current_cloud);
        icp.setInputTarget(prev_cloud_);
        icp.setMaxCorrespondenceDistance(icp_max_corr_dist_);
        icp.setMaximumIterations(icp_max_iterations_);
        icp.setTransformationEpsilon(icp_transform_eps_);

        CloudT aligned;
        icp.align(aligned);

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

        Eigen::Matrix4f T = icp.getFinalTransformation();

        double dx = T(0, 3);
        double dy = T(1, 3);
        double dtheta = std::atan2(T(1, 0), T(0, 0));

        double cos_t = std::cos(theta_);
        double sin_t = std::sin(theta_);

        x_     += dx * cos_t - dy * sin_t;
        y_     += dx * sin_t + dy * cos_t;
        theta_  = normalizeAngle(theta_ + dtheta);

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

        for (auto &c : odom.pose.covariance) c = 0.0;
        odom.pose.covariance[0]  = 0.02;
        odom.pose.covariance[7]  = 0.02;
        odom.pose.covariance[35] = 0.05;

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
