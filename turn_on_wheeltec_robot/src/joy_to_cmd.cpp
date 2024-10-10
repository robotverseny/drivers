#include <cmath>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/joy.hpp>

using std::placeholders::_1;

class JoyToCmdVel : public rclcpp::Node
{
public:
    JoyToCmdVel() : Node("joy_to_cmd_vel")
    {
        this->declare_parameter<std::string>("joy_topic", "joy");
        this->declare_parameter<std::string>("cmd_vel_topic", "cmd_vel");
        this->declare_parameter<double>("max_linear_vel", 1.5);
        this->declare_parameter<double>("max_angular_vel", 1.5);
        this->get_parameter("joy_topic", joy_topic_);
        this->get_parameter("cmd_vel_topic", cmd_vel_topic_);
        this->get_parameter("max_linear_vel", max_linear_vel_);
        this->get_parameter("max_angular_vel", max_angular_vel_);

        publisher_ = this->create_publisher<geometry_msgs::msg::Twist>(cmd_vel_topic_, 10);
        subscription_ = this->create_subscription<sensor_msgs::msg::Joy>(joy_topic_, 10, std::bind(&JoyToCmdVel::joy_callback, this, _1));

        RCLCPP_INFO(this->get_logger(), "Node 'joy_to_cmd_vel' started");
        RCLCPP_INFO_STREAM(this->get_logger(), "Subscribed to '" << joy_topic_ << "', publishing to '" << cmd_vel_topic_ << "'");
    }

private:
    void joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg)
    {
        auto cmd_msg = geometry_msgs::msg::Twist();
        cmd_msg.linear.x = max_linear_vel_ * msg->axes[3];
        cmd_msg.angular.z = max_angular_vel_ * msg->axes[0];
        publisher_->publish(cmd_msg);
    }

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr subscription_;
    std::string joy_topic_;
    std::string cmd_vel_topic_;
    double max_linear_vel_;
    double max_angular_vel_;
};
int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<JoyToCmdVel>());
    rclcpp::shutdown();
    return 0;
}