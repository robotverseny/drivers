/*************************************************************************************
@company: Copyright (C) 2021, Leishen Intelligent System, WHEELTEC (Dongguan) Co., Ltd
@product: LSn10
@filename: lsn10.h
@brief:
@version:       date:       author:            comments:
@v2.0           22-4-12      fu,Tues          ROS2
*************************************************************************************/
#ifndef LSN10_H
#define LSN10_H
#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/thread/thread.hpp>
#include <string>

#include <vector> 

#include "rclcpp/rclcpp.hpp"
#include <sensor_msgs/msg/laser_scan.hpp>
#include "lsn10/lsiosr.h"

namespace ls {

typedef struct {
    double degree;
    double range;
    int intensity;
} ScanPoint;

class LSN10 : public rclcpp::Node
{
public:
  LSN10();
  ~LSN10();
    /**
    * 实例化雷达
    * port: 串口号，
    * baud_rate: 波特率 460800
    */
    static LSN10* instance();

    /**
    * 获取雷达数据
    * poins: 雷达点的数据。类型为ScanPoint数组
    */

    int getScan(std::vector<ScanPoint> &points, rclcpp::Time & scan_time, float &scan_duration);
    /**
    * 获取软件版本号
    * version: 返回版本号
    */
    int getVersion(std::string &version);

    uint8_t CalCRC8(unsigned char * p, int len);

    int getRate();
	
  private:
    //void initParam(); delete
    void recvThread();
    void pubScanThread();

    std::vector<ScanPoint> scan_points_;
    std::vector<ScanPoint> scan_points_bak_;

    LSIOSR * serial_;
    boost::thread *recv_thread_;
    boost::thread *pubscan_thread_;
    boost::mutex pubscan_mutex_;
    boost::mutex mutex_;
    boost::condition_variable pubscan_cond_;

    bool is_shutdown_;    // shutdown recvthread
    int data_len_;
    int points_size_;
    int count_sum;
  	double min_range;
    double max_range;
    double angle_disable_min;
    double angle_disable_max;
	double angle_able_min;
    double angle_able_max;
    //ros::Time pre_time_;
    //ros::Time time_;
    rclcpp::Time pre_time_;
    rclcpp::Time time_;

    //ros::Publisher device_pub;


    std::string serial_port_;
    int baud_rate_;
    std::string scan_topic_;
    std::string frame_id_;
	
    //ros::NodeHandle n_;
    //ros::Publisher pub_;
    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr pub_;
};

}
#endif
