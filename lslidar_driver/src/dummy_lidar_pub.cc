#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <random>

class DummyLaserScannerNode : public rclcpp::Node {
public:
    DummyLaserScannerNode() : Node("random_laser_scanner_node") {
		this->declare_parameter<std::string>("frame_id", "laser_link");
		this->declare_parameter<std::string>("scan_topic", "/scan");

		this->get_parameter("frame_id", frame_id);
		this->get_parameter("scan_topic", scan_topic);

        scan_publisher_ = this->create_publisher<sensor_msgs::msg::LaserScan>(scan_topic, 10);
        // pointcloud_publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(pointcloud_topic, 10);
        
        timer_ = this->create_wall_timer(std::chrono::milliseconds(100), std::bind(&DummyLaserScannerNode::publish_random_scan, this));
    }

private:
    void publish_random_scan() {
        auto scan_msg = sensor_msgs::msg::LaserScan();
        scan_msg.header.stamp = this->now();
        scan_msg.header.frame_id = frame_id;
        scan_msg.angle_min = -1.57;
        scan_msg.angle_max = 1.57;
        scan_msg.angle_increment = 0.05;
        scan_msg.time_increment = 0.0;
        scan_msg.scan_time = 0.1;
        scan_msg.range_min = 0.0;
        scan_msg.range_max = 10.0;

        int num_readings = static_cast<int>((scan_msg.angle_max - scan_msg.angle_min) / scan_msg.angle_increment);
        scan_msg.ranges.resize(num_readings);
        scan_msg.intensities.resize(num_readings);

        std::random_device rd;
        std::mt19937 gen(rd());
        std::uniform_real_distribution<> dis(0.0, 10.0);

        for (int i = 0; i < num_readings; ++i) {
            scan_msg.ranges[i] = dis(gen);
            scan_msg.intensities[i] = dis(gen);
        }

        scan_publisher_->publish(scan_msg);
    }

    std::string frame_id = "laser_link";
    std::string scan_topic = "/scan";
    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr scan_publisher_;
    // rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DummyLaserScannerNode>());
    rclcpp::shutdown();
    return 0;
}