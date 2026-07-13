#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

class wallFollower:public rclcpp::Node
{
public:
    wallFollower():Node("wall_follower")
    {
        cmd_vel_pub_=this->create_publisher<geometry_msgs::msg::TwistStamped>("/cmd_vel",10);
        laser_subscriber_=this->create_subscription<sensor_msgs::msg::LaserScan>("/scan",10, std::bind(&wallFollower::scan_callback, this, std::placeholders::_1));
        last_time_=this->now();
    }
private:
    void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr scan)
    {
        double distance=std::round(scan->ranges[90]*100.0)/100.0;
        double sixty=scan->ranges[60];
        double one_two_zero=scan->ranges[120];

        // work out real elapsed time since the last callback
        rclcpp::Time now=this->now();   //current time
        double dt=(now-last_time_).seconds();  //dt how much time has passed between last and current callback
        last_time_=now;
        if (dt<=0.0) dt=0.05; // guard against first-call or clock weirdness

        geometry_msgs::msg::TwistStamped cmd;

        double target=1.0;
        double linear_speed=0.15;

        // gains, tune these by testing
        double kp_distance=0.5, ki_distance=0.05, kd_distance=0.1;
        double kp_heading=0.4,  ki_heading=0.02,  kd_heading=0.05;
 
        // errors
        double distance_error=target-distance;   // positive means too close
        double heading_error=sixty-one_two_zero;  // negative means nose angled toward wall

        // integral: accumulate error over time
        distance_integral_+=distance_error*dt;
        heading_integral_+=heading_error*dt;

        // derivative: how fast the error is changing
        double distance_derivative=(distance_error-prev_distance_error_)/dt;
        double heading_derivative=(heading_error-prev_heading_error_)/dt;

        prev_distance_error_=distance_error;
        prev_heading_error_=heading_error;

        // p= kp*error  I=ki*integral D=kp*derivative   (Intergral=Intergal+error*dt,  derivative=(error-previous_error)/dt)
        double distance_term = kp_distance*distance_error + ki_distance*distance_integral_ + kd_distance*distance_derivative; 
        double heading_term  = kp_heading*heading_error  + ki_heading*heading_integral_  + kd_heading*heading_derivative;

        double angular_correction = -distance_term + heading_term; // -distance-term because we want to steer opposite

        // clamp so turning never becomes unrealistically sharp
        double max_angular=0.3;
        if (angular_correction>max_angular) angular_correction=max_angular;
        if (angular_correction<-max_angular) angular_correction=-max_angular;

        cmd.twist.linear.x=linear_speed;
        cmd.twist.angular.z=angular_correction;

        printf("dist:%.2f | dist_err:%.2f | head_err:%.2f | dt:%.3f | angular:%.2f\n",
               distance,distance_error,heading_error,dt,cmd.twist.angular.z);

        cmd_vel_pub_->publish(cmd);
    }

    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr cmd_vel_pub_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_subscriber_;

    rclcpp::Time last_time_;
    double distance_integral_=0.0;
    double heading_integral_=0.0;
    double prev_distance_error_=0.0;
    double prev_heading_error_=0.0;
};

int main(int argc,char *argv[])
{   
    rclcpp::init(argc,argv);
    auto node=std::make_shared<wallFollower>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}