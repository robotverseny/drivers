#include <cmath>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <ackermann_msgs/msg/ackermann_drive_stamped.hpp>

using std::placeholders::_1;

class CmdVelToAckermannDrive : public rclcpp::Node
{
public:
    CmdVelToAckermannDrive() : Node("cmd_vel_to_ackermann_drive")
    {
        this->declare_parameter<std::string>("cmd_vel_topic", "cmd_vel");
        this->declare_parameter<std::string>("ackermann_cmd_topic", "/ackermann_cmd");
        this->declare_parameter<std::string>("frame_id", "odom_combined");
        this->declare_parameter<double>("wheelbase", 0.3187);
        this->get_parameter("cmd_vel_topic", cmd_vel_topic_);
        this->get_parameter("ackermann_cmd_topic", ackermann_cmd_topic_);
        this->get_parameter("frame_id", frame_id_);
        this->get_parameter("wheelbase", wheelbase_);

        //frame_id_ = "odom_combined";
        //wheelbase_ = 0.3187;  // meters wheeltec roboworks rosbot mini
        cmd_angle_instead_rotvel_ = false; //

        publisher_ = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>(ackermann_cmd_topic_, 10);
        subscription_ = this->create_subscription<geometry_msgs::msg::Twist>(cmd_vel_topic_, 10, std::bind(&CmdVelToAckermannDrive::cmd_callback, this, _1));
        timer_ = this->create_wall_timer(std::chrono::milliseconds(100), std::bind(&CmdVelToAckermannDrive::timer_callback, this));

        RCLCPP_INFO(this->get_logger(), "Node 'cmd_vel_to_ackermann_drive' started");
        RCLCPP_INFO_STREAM(this->get_logger(), "Subscribed to '" << cmd_vel_topic_ << "', publishing to '" << ackermann_cmd_topic_ << "'");
    }

private:
    void cmd_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
        double v = msg->linear.x;
        double steering;

        if (cmd_angle_instead_rotvel_)
        {
            steering = msg->angular.z;
        }
        else
        {
            steering = convert_trans_rot_vel_to_steering_angle(v, msg->angular.z, wheelbase_);
        }

        RCLCPP_INFO_ONCE(this->get_logger(), "ackermann msg: steering %.2f, speed %.2f", steering, v);

        ackermann_msg.header.stamp = this->get_clock()->now();
        ackermann_msg.header.frame_id = frame_id_;
        ackermann_msg.drive.steering_angle = steering;
        ackermann_msg.drive.speed = v;
        if(steering > 0.00000001 || steering < -0.00000001)
        {
            ackermann_msg.drive.steering_angle_velocity = 0.5;
        }
        else
        {
            ackermann_msg.drive.steering_angle_velocity = 0.0;
        }
    }

    double convert_trans_rot_vel_to_steering_angle(double v, double omega, double wheelbase)
    {
        if (omega == 0)
        {
            return 0;
        }

        double radius = v / omega;
        return std::atan(wheelbase / radius);
    }

    void timer_callback()
    {
        RCLCPP_INFO_ONCE(this->get_logger(), "Node 'cmd_vel_to_ackermann_drive' is running");
        publisher_->publish(ackermann_msg);
    }

    rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr publisher_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr subscription_;
    std::string frame_id_, cmd_vel_topic_, ackermann_cmd_topic_;
    rclcpp::TimerBase::SharedPtr timer_;
    ackermann_msgs::msg::AckermannDriveStamped ackermann_msg;
    double wheelbase_ = 0.3187;
    bool cmd_angle_instead_rotvel_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CmdVelToAckermannDrive>());
    rclcpp::shutdown();
    return 0;
}