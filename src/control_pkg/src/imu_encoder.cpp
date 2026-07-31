#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2_msgs/msg/tf_message.hpp"
#include "nav_msgs/msg/odometry.hpp"





class ImuEncoderFusion: public rclcpp::Node
{
public:
        ImuEncoderFusion():Node("imu_encoder_fusion")
        {
            joint_state_sub_=this->create_subscription<sensor_msgs::msg::JointState>("/joint_states",10, std::bind(&ImuEncoderFusion::encoder_callback, this, std::placeholders::_1));
            imu_sub_=this->create_subscription<sensor_msgs::msg::Imu>("/imu", 10, std::bind(&ImuEncoderFusion::imu_callback, this, std::placeholders::_1));
            actual_info_sub_=this->create_subscription<tf2_msgs::msg::TFMessage>("/gt_tf", 10, std::bind(&ImuEncoderFusion::actual_info_callback, this, std::placeholders::_1));
            timer_=this->create_wall_timer( std::chrono::milliseconds(20), std::bind(&ImuEncoderFusion::timer_callback, this));
            odom_pub_=this->create_publisher<nav_msgs::msg::Odometry>("/wheel_odom",10);
        }
private:
        
        bool first_encoder=true;
        double left_wheel_=0.0;
        double right_wheel_=0.0;
        
        double left_wheel_old=0.0;
        double right_wheel_old=0.0;

        double angular_velocity_z_=0.0;

        double gt_x_=0.0;
        double gt_y_=0.0;
        double gt_yaw_=0.0;

        double wheel_base=0.175;

        double theta_enc_rad=0.0;  // heading from wheel encoders
        double theta_imu_rad=0.0;  // heading from imu gyro

        double theta_fused_rad=0.0;

        double dt=0.02; //as timer callback is 20ms

        double x=0.0;
        double y=0.0;


        void encoder_callback(const sensor_msgs::msg::JointState::SharedPtr msg)
        {
            left_wheel_=msg->position[0];
            right_wheel_=msg->position[1];

            // double left_wheel_vel=msg->velocity[0];
            // double right_wheel_vel=msg->velocity[1];

            // printf("Left_wheel_pos:%.2f | Right_wheel_pos:%.2f\n",left_wheel_,right_wheel_);


        }

        void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg)
        {
            // tf2::Quaternion q(
            //     msg->orientation.x,
            //     msg->orientation.y,
            //     msg->orientation.z,
            //     msg->orientation.w);

            // double roll,pitch,yaw;
            // tf2::Matrix3x3(q).getRPY(roll,pitch,yaw);
            // double yaw_deg=yaw*180.0/M_PI; 

           angular_velocity_z_=msg->angular_velocity.z;
            
        //    printf("ang_velocity:%.2f\n",angular_velocity_z_);

        //    double linear_acc_x=msg->linear_acceleration.x;
        }

        void actual_info_callback(const tf2_msgs::msg::TFMessage::SharedPtr msg)
        {
            const auto &tf= msg->transforms[0];

            gt_x_=tf.transform.translation.x;
            gt_y_=tf.transform.translation.y;

            double qx=tf.transform.rotation.x;
            double qy=tf.transform.rotation.y;
            double qz=tf.transform.rotation.z;
            double qw=tf.transform.rotation.w;

            tf2::Quaternion q(qx,qy,qz,qw);
            double roll,pitch,yaw;
            tf2::Matrix3x3(q).getRPY(roll,pitch,yaw);
            gt_yaw_=yaw*180.0/M_PI;
            // printf("(x,y):%.2f,%.2f | Yaw:%.2f ",gt_x_,gt_y_,gt_yaw_);
        }

        double normalizeAngle(double angle)
        {
            if (angle> M_PI)
            {
                angle-=2.0*M_PI;
            }
            else if (angle < -M_PI)
            {
                angle += 2.0*M_PI;
            }
            return angle;
        }

        double toDegrees(double angle)
        {
            angle=angle*180.0/M_PI;
            return angle;
        }

        void timer_callback()
        {   

            if (first_encoder)
            {
                left_wheel_old=left_wheel_;
                right_wheel_old=right_wheel_;
                first_encoder=false;
                return;
            }

            double delta_left=left_wheel_-left_wheel_old;
            double delta_right=right_wheel_-right_wheel_old;

            double left_distance=delta_left*0.06;
            double right_distance=delta_right*0.06;
            double distance=(left_distance+right_distance)/2.0;

            double delta_theta=(right_distance-left_distance)/wheel_base;  //how much turned between callback

            theta_enc_rad+=delta_theta;
            theta_enc_rad=normalizeAngle(theta_enc_rad);
            
            double theta_enc_deg=toDegrees(theta_enc_rad);
            
            double delta_theta_imu = angular_velocity_z_ * dt;  //how much robot rotated between the callbacks (here 0.02sec)
            theta_imu_rad+=delta_theta_imu; //add timy rotations to get total rotations
            theta_imu_rad=normalizeAngle(theta_imu_rad);
            double theta_imu_deg=toDegrees(theta_imu_rad);

            
            double alpha=0.50;

            theta_fused_rad=theta_fused_rad+delta_theta_imu;   // Prediction  adding gyro reading to previous fused estimate
            theta_fused_rad=normalizeAngle(theta_fused_rad);
            theta_fused_rad=( alpha*theta_fused_rad + ( (1.0-alpha)*theta_enc_rad));  // Correction 
            theta_fused_rad=normalizeAngle(theta_fused_rad);
            double theta_fused_deg=toDegrees(theta_fused_rad);

            double error_normal=theta_imu_deg-theta_enc_deg;
            double error_fused=gt_yaw_-theta_fused_deg;
            

 
            // double 
            x+=distance*cos(theta_enc_rad);
            y+=distance*sin(theta_enc_rad);
            // printf("left:%.2f | right:%.2f | old_left:%.2f | old_right:%.2f | Dleft:%.2f | Dright:%.2f | ang_vel:%.2f | real (x,y):%.2f,%.2f | cal(x,y):%.2f,%.2f| yaw:%.2f | distance:%.4f | dtheta:%.2f | theta:%.2f\n",left_wheel_, right_wheel_, left_wheel_old, right_wheel_old, delta_left, delta_right, angular_velocity_z_, gt_x_ , gt_y_, x, y, gt_yaw_, distance, delta_theta, theta_degree);
            printf(" ang_vel:%.2f | real (x,y):%.2f,%.2f | cal(x,y):%.2f,%.2f| yaw_real:%.2f | theta_enc:%.2f | theta_imu:%.2f |theta_fused:%.2f | error_n:%.2f | error_f:%.2f\n",angular_velocity_z_, gt_x_ , gt_y_, x, y, gt_yaw_,theta_enc_deg, theta_imu_deg,theta_fused_deg, error_normal,error_fused);
            left_wheel_old=left_wheel_;
            right_wheel_old=right_wheel_;

            nav_msgs::msg::Odometry odom;
            odom.header.stamp=this->get_clock()->now();

            odom.header.frame_id="odom";
            odom.child_frame_id="base_link";

            odom.pose.pose.position.x=x;
            odom.pose.pose.position.y=y;
            odom.pose.pose.position.z=0.0;

            tf2::Quaternion a;
            a.setRPY(0,0,theta_enc_rad);

            odom.pose.pose.orientation.x=a.x();
            odom.pose.pose.orientation.y=a.y();
            odom.pose.pose.orientation.z=a.z();
            odom.pose.pose.orientation.w=a.w();
            odom_pub_->publish(odom);


            
        }


        rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
        rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
        rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr actual_info_sub_;
        rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
        rclcpp::TimerBase::SharedPtr timer_;

    
};

int main(int argc,char *argv[])
{
    rclcpp::init(argc,argv);
    auto node=std::make_shared<ImuEncoderFusion>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;

}
