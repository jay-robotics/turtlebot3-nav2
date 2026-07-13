#include <algorithm>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "std_msgs/msg/float64.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include <string>


class waypointFollower: public rclcpp::Node 
{
    public:
    waypointFollower():Node("waypoint_follower")
    {
        cmd_pub_=this->create_publisher<geometry_msgs::msg::TwistStamped>("/cmd_vel",10);
        odom_sub_=this->create_subscription<nav_msgs::msg::Odometry>("/odom",10,std::bind(&waypointFollower::odom_callback,this,std::placeholders::_1));
        p_pub_=this->create_publisher<std_msgs::msg::Float64>("/pid/p",10);
        i_pub_=this->create_publisher<std_msgs::msg::Float64>("/pid/i",10);
        d_pub_=this->create_publisher<std_msgs::msg::Float64>("/pid/d",10);
        angular_speed_pub_=this->create_publisher<std_msgs::msg::Float64>("/pid/angular_speed",10);
        angular_error_pub_=this->create_publisher<std_msgs::msg::Float64>("/pid/angular_error",10);
        linear_error_pub_=this->create_publisher<std_msgs::msg::Float64>("/pid/linear_error",10);
        linear_speed_pub_=this->create_publisher<std_msgs::msg::Float64>("/pid/linear_speed",10);
        last_time=this->now();
    }
    private:
    std::string flag="rotate";
    int current_waypoint=0;
    std::vector<std::pair<double,double>> waypoints={
            {0.0, -3.0},
            {3.0, -3.0},
            {3.0,  3.0},
            {-3.0, 3.0},
            {-3.0, -3.0}
    };
    

    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        tf2::Quaternion q(
            msg->pose.pose.orientation.x,
            msg->pose.pose.orientation.y,
            msg->pose.pose.orientation.z,
            msg->pose.pose.orientation.w
        
        );

        rclcpp::Time now=this->now();
        double dt=(now-last_time).seconds();
        last_time=now;
        if (dt<=0.0) dt=0.05;



        // points=[(0,0) -> (0,-3),(3,-3),(3,3),(-3,3),(-3,-3)]
        double roll,pitch,yaw;
        tf2::Matrix3x3(q).getRPY(roll,pitch,yaw);
        double current_yaw_deg=yaw*180.0/M_PI;

        double current_x=msg->pose.pose.position.x;
        double current_y=msg->pose.pose.position.y;


        
        double goal_x;
        double goal_y;
        
        if (current_waypoint<waypoints.size())
        {
            goal_x=waypoints[current_waypoint].first;
            goal_y=waypoints[current_waypoint].second;
            
        }
        
        else
        {
            printf("all goals reaches\n");
            return;
        }

        double dx=goal_x-current_x;
        double dy=goal_y-current_y;
        double angle_to_goal=(std::atan2(dy,dx))*180/M_PI;

        geometry_msgs::msg::TwistStamped cmd;
        
        double kp_ang=0.05, kd_ang=0.00, ki_ang=0.00;
        double kp_dis=0.8, kd_dis=0.00, ki_dis=0.00;
        
        double angular_error=angle_to_goal-current_yaw_deg; //both angles should be in same convention/range,here both are in -180 to 180 range
        if (angular_error>180)
        angular_error-=360;
       if (angular_error<-180)
           angular_error+=360;
        double linear_error=std::sqrt(dx*dx+dy*dy);

        //accumulate distance for intergal
        angular_integral+=angular_error*dt;
        linear_integral+=linear_error*dt;

        //derivative
        double angular_derivative=(angular_error-prev_angular_error)/dt;
        double linear_derivative=(linear_error-prev_linear_error)/dt;
        prev_angular_error=angular_error;
        prev_linear_error=linear_error;
        double p=kp_ang*angular_error;
        double i=ki_ang*angular_integral;
        double d=kd_ang*angular_derivative;

        double angular_speed=p+i+d;
        double linear_speed= kp_dis*linear_error + ki_dis*linear_integral + kd_dis*linear_derivative;

        double angular_speed_clamped = std::clamp(angular_speed, -0.3, 0.3);
        linear_speed = std::clamp(linear_speed, -1.0, 1.0);

        std_msgs::msg::Float64 p_msg;
        p_msg.data = p;
        p_pub_->publish(p_msg);

        std_msgs::msg::Float64 i_msg;
        i_msg.data = i;
        i_pub_->publish(i_msg);

        std_msgs::msg::Float64 d_msg;
        d_msg.data = d;
        d_pub_->publish(d_msg);

        std_msgs::msg::Float64 angular_speed_msg;
        angular_speed_msg.data = angular_speed_clamped;
        angular_speed_pub_->publish(angular_speed_msg);

        std_msgs::msg::Float64 angular_error_msg;
        angular_error_msg.data = angular_error;
        angular_error_pub_->publish(angular_error_msg);

        std_msgs::msg::Float64 linear_error_msg;
        linear_error_msg.data = linear_error;
        linear_error_pub_->publish(linear_error_msg);

        std_msgs::msg::Float64 linear_speed_msg;
        linear_speed_msg.data = linear_speed;
        linear_error_pub_->publish(linear_speed_msg);




        // if (angular_speed>0.1)
        // {
        //     angular_speed=0.2;
        // }
        // else if (angular_speed<-0.1)
        // {
        //     angular_speed=-0.2;
        // }

        cmd.header.stamp=this->now();
        cmd.header.frame_id="base_link";


        // if (angular_error>=-0.1)
        //     {
        //         cmd.twist.linear.x=linear_speed;
        //         cmd.twist.angular.z=0.0;
        //         printf("current:%.2f | Goal:%.2f | ang_error:%.2f | ang_speed:%.2f | g_error:%.2f | l_speed:%.2f\n",current_yaw_deg,angle_to_goal,angular_error,0.0,linear_error,linear_speed);
        //     }
        // else
        // {
        //     cmd.twist.angular.z=angular_speed_clamped;
        //     printf("current:%.2f | Goal:%.2f | ang_error:%.2f | ang_speed:%.2f | ang_speed_clamped:%.2f | g_error:%.2f | l_speed:%.2f\n",current_yaw_deg,angle_to_goal,angular_error,angular_speed,angular_speed_clamped,linear_speed);
        // }
        printf("entered if else\n");
        if (flag=="rotate")
        { 
            printf("entered rotate\n");
            if (std::abs(angular_error)>0.5)
            {
                cmd.twist.angular.z=angular_speed_clamped;
                cmd.twist.linear.x=0.0;
                printf("current:%.2f | Goal:%.2f | ang_error:%.2f | ang_speed:%.2f | ang_speed_clamped:%.2f | goal:%.2f,%.2f\n",current_yaw_deg, angle_to_goal, angular_error, angular_speed, angular_speed_clamped,goal_x,goal_y);
            }
            else
            {
                cmd.twist.angular.z=0.0;
                printf("current:%.2f | Goal:%.2f | ang_error:%.2f | ang_speed:%.2f | ang_speed_clamped:%.2f | goal:%.2f,%.2f\n",current_yaw_deg, angle_to_goal, angular_error, angular_speed, angular_speed_clamped,goal_x,goal_y);
                flag="straight";
                printf("flag changes from rotate to straight\n");
            }
        } 
        else if(flag=="straight")
        {
            printf("enter straight\n");
            if (linear_error>0.05 || std::abs(angular_error)>0.5)
            {
                cmd.twist.angular.z=angular_speed_clamped;
                cmd.twist.linear.x=linear_speed;
                printf("current:%.2f | Goal:%.2f | ang_error:%.2f | ang_speed:%.2f | ang_speed_clamped:%.2f |  l_er:%.2f | ls:%.2f | goal:%2.f,%2.f \n",current_yaw_deg, angle_to_goal, angular_error, angular_speed, angular_speed_clamped,linear_error,linear_speed,goal_x,goal_y);
            }
            else if (linear_error<0.05 && std::abs(angular_error)<0.05)
            {
                cmd.twist.linear.x=0.0;
                cmd.twist.angular.z=0.0;
                printf("current:%.2f | Goal:%.2f | ang_error:%.2f | ang_speed:%.2f | ang_speed_clamped:%.2f | lr:%.2f | ls:%.2f | goal:%.2f,%.2f\n",current_yaw_deg, angle_to_goal, angular_error, angular_speed, angular_speed_clamped,linear_error,linear_speed,goal_x,goal_y);
                printf("flag changes from straight to rotate\n");
                flag="rotate";
                current_waypoint++;
            }
        }
        
        
        // printf("current:%.2f | Goal:%.2f | ang_error:%.2f | ang_speed:%.2f | ang_speed_clamped:%.2f |p:%.2f | \n",current_yaw_deg, angle_to_goal, angular_error, angular_speed, angular_speed_clamped,p);
        // printf("current:%.2f | Goal:%.2f | ang_error:%.2f | ang_speed:%.2f | ang_speed_clamped:%.2f |p:%.2f|i:%.2f|d:%.2f\n",current_yaw_deg, angle_to_goal, angular_error, angular_speed, angular_speed_clamped,p,i,d);
        cmd_pub_->publish(cmd);

    }


    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr cmd_pub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr p_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr i_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr d_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr angular_speed_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr angular_error_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr linear_error_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr linear_speed_pub_;
    rclcpp::Time last_time;
    double angular_integral=0.0;
    double linear_integral=0.0;
    double prev_linear_error=0.0;
    double prev_angular_error=0.0;




};

int main(int argc,char *argv[])
{
    rclcpp::init(argc,argv);
    auto node=std::make_shared<waypointFollower>();
    rclcpp::spin(node);
    rclcpp::shutdown();
}


