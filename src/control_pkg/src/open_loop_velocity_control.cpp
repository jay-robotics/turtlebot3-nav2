#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include <memory>
#include <iostream>
#include <cmath>

class cmdvelpublisher: public rclcpp::Node
{

public:
    cmdvelpublisher():Node("cmd_vel_publisher")
    {
        cmd_vel_pub_= this->create_publisher<geometry_msgs::msg::TwistStamped>("/cmd_vel",10);
        // timer_=this->create_wall_timer( std::chrono::milliseconds(100), std::bind(&cmdvelpublisher::publish_cmd_vel,this));
        odom_subscriber_=this->create_subscription<nav_msgs::msg::Odometry>("/odom",10,std::bind(&cmdvelpublisher::odom_callback, this, std::placeholders::_1));
        RCLCPP_INFO(this->get_logger(), "Node created");
        target_angle=90.0;
        // error;   //>0 -> anticlk, <0->clk, ==0 stop

    }
private:
    void publish_cmd_vel()
    {
        RCLCPP_INFO(this->get_logger()," enter publisher");
        std::cout << "Publishing velocity command!" << std::endl;
        
        geometry_msgs::msg::TwistStamped cmd;
        cmd.header.stamp = this->now();
        cmd.header.frame_id = "base_link";
        
        cmd.twist.linear.x=0.2;
        cmd.twist.linear.y=0.0;
        cmd.twist.linear.z=0.0;

        cmd.twist.angular.x=0.0;
        cmd.twist.angular.y=0.0;
        cmd.twist.angular.z=0.0;
        
        // cmd_vel_pub_->publish(cmd);but
        // RCLCPP_INFO(this->get_logger(),"published");
        // std::cout << "Message sent with x=" << cmd.twist.linear.x << std::endl;
    }

    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        tf2::Quaternion q(
            msg->pose.pose.orientation.x,
            msg->pose.pose.orientation.y,
            msg->pose.pose.orientation.z,
            msg->pose.pose.orientation.w);

        double roll,pitch,yaw ; 
        tf2::Matrix3x3(q).getRPY(roll,pitch,yaw); //get angles in rpy in radians
        double yaw_deg=yaw*180.0/M_PI;  //convert to degrees=radians*180/pi
        if (yaw_deg<0)
        {
            yaw_deg+=360;
        }

        error=target_angle-yaw_deg;
        if (error>180)
         error-=360;
        if (error<-180)
            error+=360;
        geometry_msgs::msg::TwistStamped cmd;
        cmd.header.stamp = this->now();
        cmd.header.frame_id = "base_link";
        double speed=0.05;
        if (error>0.5)
        {cmd.twist.angular.z=speed;}
        else if (error<0.0)
        {cmd.twist.angular.z=-speed;}
        else if(error>=0 && error<=0.5)
        {cmd.twist.angular.z=0.0;}
        cmd_vel_pub_->publish(cmd);


        // RCLCPP_INFO(this->get_logger(),"Roll: %.2f, Pitch: %.2f, yaw: %.2f", roll, pitch, yaw);
        RCLCPP_INFO(this->get_logger(),"target_angle: %.2f,yaw degrees: %.2f,error: %.2f",target_angle,yaw_deg,error); //%=value will eb inserted her, f=float or double, .2=display exactly 2 digits after decimal point


    }
    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr cmd_vel_pub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscriber_;
    rclcpp::TimerBase::SharedPtr timer_;
    double target_angle,error;


};

int main(int argc,char *argv[])
{
    rclcpp::init(argc,argv);
    auto node=std::make_shared<cmdvelpublisher>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;

}
