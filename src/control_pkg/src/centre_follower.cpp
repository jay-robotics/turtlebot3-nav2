#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include <algorithm>

class centreFollower: public rclcpp::Node
{
public:
    centreFollower():Node("centre_follower")
    {
        cmd_vel_pub_=this->create_publisher<geometry_msgs::msg::TwistStamped>("/cmd_vel",10);
        lidar_subs_=this->create_subscription<sensor_msgs::msg::LaserScan>("/scan", 10, std::bind(&centreFollower::scan_callback, this, std::placeholders::_1));
        last_time_=this->now();
    }
private:

    double fix_inf(double value)
    {
        if (std::isinf(value))
            return 12.0;
        return value;
    }

    void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
    {

        double left_120=msg->ranges[120];
        double right_240=msg->ranges[240];
        double left_dist=fix_inf(msg->ranges[90]);
        double left_60=fix_inf(msg->ranges[60]);

        
        //average
        double sum_60_window=0.0;
        for (int i=56;i<=65;i++)
        {
            if (std::isinf(msg->ranges[i]))
                msg->ranges[i]=12.0;
            sum_60_window+=msg->ranges[i];
        }
        double average_60=sum_60_window/10.0;
        double alpha=0.8;
        double filter_60=(alpha*prev_avg_60)+((1-alpha)*average_60);
        prev_avg_60=average_60;
        
        
        double sum_300_window=0.0;
        for (int i=296;i<=305;i++)
        {
            if (std::isinf(msg->ranges[i]))
                msg->ranges[i]=12.0;
            sum_300_window+=msg->ranges[i];
        }
        double avg_300=sum_300_window/10.0;
        double filter_300=(alpha*prev_avg_300)+((1-alpha)*avg_300);
        prev_avg_300=avg_300;

        // double right_dist=fix_inf(msg->ranges[270]);
        // double right_300=fix_inf(msg->ranges[300]);
        rclcpp::Time now=this->now();
        double dt=(now-last_time_).seconds();
        last_time_=now;
        if (dt<=0) dt=0.05;


 
        // double dist_error=left_dist-right_dist;
        double front_error_60_300=filter_60-filter_300;
        // double front_error_60_300=average_60-avg_300;

    //    double dist_error=left_dist-right_dist;
//         double front_error_60_300=left_60-right_300;
        double distance_derivative=(front_error_60_300-previous_distance_error)/dt;
        distance_derivative=std::clamp(distance_derivative,-5.0,5.0);
        
        previous_distance_error=front_error_60_300;
        

        geometry_msgs::msg::TwistStamped cmd;
        double kp=0.5;
        double kd=0.05;
        // double angular_speed=kp*front_error_60_300 ;
        double angular_speed=kp*front_error_60_300 + kd*distance_derivative;
        angular_speed=std::clamp(angular_speed,-0.2,0.2);
        double n_angular_speed=angular_speed;
        // double max_range=0.1;
        // double delta=std::clamp(angular_speed-prev_angular_speed,-max_range,max_range);
        // angular_speed=prev_angular_speed+delta;
        // prev_angular_speed=angular_speed;
        
        // printf("Left dist:%.2f,right dist:%.2f | 60:%.2f,240:%.2f | 120:%.2f,300:%.2f | dist_error:%.2f | diag_error:%.2f \n",left_dist,right_dist,left_60,right_240,left_120,right_300,dist_error,front_error_60_300);
        double desired_radius=0.2;
        double cruise_speed=0.3;
        // double turn_speed=std::abs(angular_speed)*desired_radius;
        double new_linear;
        double ang_alpha=0.5;
        // if (angular_speed==0.2 || angular_speed==-0.2 )
        // {     
        //     angular_speed=ang_alpha*angular_speed;
        //     // linear_speed=0.1;
        // }
        if (std::abs(angular_speed) < 0.30) {
            new_linear = cruise_speed;
        } else {
            new_linear = std::abs(angular_speed) * desired_radius;
            new_linear = std::clamp(new_linear, 0.08, cruise_speed);
        }

        double linear_speed=0.2;
        
        if (n_angular_speed==0.2 || n_angular_speed==-0.2 )
        {     
            n_angular_speed=ang_alpha*n_angular_speed;
            linear_speed=0.1;
        }
        else if (n_angular_speed==(ang_alpha*0.2) || n_angular_speed==(ang_alpha*(-0.2)))
        {
            // printf("as_pos:%.2f neg:%.2f\n",(ang_alpha*0.2)*0.75,(ang_alpha*(-0.2))*0.75);
            linear_speed=0.1;
        }
        else
        {
            linear_speed=linear_speed;
        }
        printf("Left_60:%.2f | Right_300:%.2f | Front error:%.2f | D:%.2f | front+D:%.2f | n_ang_speed:%.2f | ang_speed:%.2f | lv:%.2f\n",average_60,avg_300,front_error_60_300,distance_derivative,front_error_60_300+distance_derivative,n_angular_speed,angular_speed,linear_speed);
        // printf("Left_60:%.2f | Right_300:%.2f | Front error:%.2f | D:%.2f | front+D:%.2f | ang_speed:%.2f | lv:%.2f filter_60:%.2f filter_300:%.2f\n",average_60,avg_300,front_error_60_300,distance_derivative,front_error_60_300+distance_derivative,angular_speed,linear_speed,filter_60,filter_300);
        cmd.twist.linear.x=linear_speed;
        cmd.twist.angular.z=n_angular_speed;
        
        cmd_vel_pub_->publish(cmd);




    }
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr lidar_subs_;
    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr cmd_vel_pub_;
    rclcpp::Time last_time_;
    double previous_distance_error=0.0;
    double previous_derivative=0.0;
    double prev_filtered_60=0.0;
    double prev_filtered_300=0.0;
    double current_filtered=0.0;
    double prev_avg_60=0.0;
    double prev_avg_300=0.0;
    double prev_angular_speed=0.0;


};

int main(int argc, char *argv[])
{
    rclcpp::init(argc,argv);
    auto node=std::make_shared<centreFollower>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}