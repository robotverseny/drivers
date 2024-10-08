/*************************************************************************************
@company: Copyright (C) 2021, Leishen Intelligent System, WHEELTEC (Dongguan) Co., Ltd
@product: LSn10
@filename: lsn10.cpp
@brief:
@version:       date:       author:            comments:
@v2.0           22-4-12      leo,Tues          ROS2
*************************************************************************************/
#include <iostream> 

#include "lsn10/lsn10.h"
#include <stdio.h>
#include <signal.h> 

using std::placeholders::_1;
using namespace std;


namespace ls{
LSN10 * LSN10::instance()
{
  static LSN10 obj;
  return &obj;
}
 

LSN10::LSN10():rclcpp::Node ("n10lidar")
{
  int code = 0;
  //float max_range=100;
  pre_time_ = rclcpp::Node::now();
  time_ = rclcpp::Node::now();

  std::string scan_topic_ = "/scan";
  std::string frame_id_ = "laser";
  std::string serial_port_ = "/dev/wheeltec_laser";
  float min_range=0;
  float max_range=100;
  float angle_disable_min=0;
  float angle_disable_max=0;


  this->declare_parameter<std::string>("scan_t opic", "/scan");
  this->get_parameter<std::string>("scan", scan_topic_);

  this->declare_parameter<std::string>("laser", "laser");
  this->get_parameter<std::string>("laser", frame_id_);

  this->declare_parameter<std::string>("serial_port", "/dev/wheeltec_laser");
  this->get_parameter<std::string>("serial_port", serial_port_);

  this->declare_parameter<int>("baud_rate", 230400);
  this->get_parameter<int>("baud_rate", baud_rate_);

  this->declare_parameter<float>("min_range", 0);
  this->get_parameter<float>("min_range", min_range); 

  this->declare_parameter<float>("max_range", 100.0);
  this->get_parameter<float>("max_range", max_range);

  this->declare_parameter<float>("angle_disable_min", 0);
  this->get_parameter<float>("angle_disable_min", angle_disable_min);

  this->declare_parameter<float>("angle_disable_max", 0);
  this->get_parameter<float>("angle_disable_max", angle_disable_max);

  count_sum = 0;
  while(angle_disable_min<0)
	angle_disable_min+=360;
  while(angle_disable_max<0)
	angle_disable_max+=360;
  while(angle_disable_min>360)
	angle_disable_min-=360;
  while(angle_disable_max>360)
	angle_disable_max-=360;
  if(angle_disable_max == 0.0 && angle_disable_min == 0.0)
  {
	angle_able_min = 0;
	angle_able_max = 360;
  }
  else
  {
	if(angle_disable_min<angle_disable_max && angle_disable_min !=0.0)
	{
		angle_able_min = angle_disable_max;
		angle_able_max = angle_disable_min+360;
	}
	if (angle_disable_min<angle_disable_max && angle_disable_min == 0.0)
	{
		angle_able_min = angle_disable_max;
		angle_able_max = 360;
	}
	if (angle_disable_min>angle_disable_max )
	{
		angle_able_min = angle_disable_max; 
		angle_able_max = angle_disable_min; 
	}
  }
  is_shutdown_ = false;
  data_len_ = 200;
  points_size_ = 600 ;
  scan_points_.resize(points_size_);

  pub_ = create_publisher<sensor_msgs::msg::LaserScan>(scan_topic_, 3);
  serial_ = LSIOSR::instance(serial_port_, baud_rate_);
  code = serial_->init();
  if(code != 0)
  {
	printf("open_port %s ERROR !\n", serial_port_.c_str());
	rclcpp::shutdown();
	exit(0);

  }

  printf("open_port %s  OK !\n", serial_port_.c_str());
  recv_thread_ = new boost::thread(boost::bind(&LSN10::recvThread, this));
  pubscan_thread_ = new boost::thread(boost::bind(&LSN10::pubScanThread, this));
}

LSN10::~LSN10()
{
  printf("start LSN10::~LSN10()\n");

  is_shutdown_ = true;
  pubscan_thread_->interrupt();
  pubscan_thread_->join();
  pubscan_thread_ = NULL;
  delete pubscan_thread_;

  recv_thread_->interrupt();
  recv_thread_->join();
  recv_thread_ = NULL;
  delete recv_thread_;

  serial_->close();
  serial_ = NULL;
  delete serial_;
  printf("end LSN10::~LSN10()\n");
}

int LSN10::getScan(std::vector<ScanPoint> &points, rclcpp::Time &scan_time, float &scan_duration)
{
  boost::unique_lock<boost::mutex> lock(mutex_);

  points.assign(scan_points_bak_.begin(), scan_points_bak_.end());
  scan_time = pre_time_;
  scan_duration = (time_ - pre_time_).seconds();
  if(scan_duration == 0)
  {
        time_ = rclcpp::Node::now();
        scan_duration = (time_ - pre_time_).seconds();
  }
}

int LSN10::getVersion(std::string &version)
{
  version = "lsn10_v1_0";
  return 0;
}

void LSN10::recvThread()
{
  unsigned char * packet_bytes = new unsigned char [data_len_];
  int idx = 0;
  int link_time = 0;
  double degree;
  double end_degree;
  double degree_interval;
  double last_degree = 0.0;

  boost::posix_time::ptime t1,t2;
  t1 = boost::posix_time::microsec_clock::universal_time();

  while(!is_shutdown_&&rclcpp::ok()){

	int count = serial_->read(packet_bytes, 58);

	if(count <= 0) 
		link_time++;
	else
		link_time = 0;
	
	if(link_time > 10000)
	{
		serial_->close();
		int ret = serial_->init();
		if(ret < 0)
		{
			RCLCPP_WARN(get_logger(),"serial open fail");
			usleep(300000);
		}
		link_time = 0;
	}
	
	if(count <= 0) 
		continue;

	for (int i = 0; i < count; i++)
	{
		
		
		int k = packet_bytes[i];
		k < 0 ? k += 256 : k;
		int y = packet_bytes[i + 1];
		y < 0 ? y += 256 : y;
		if (k == 0xA5 && y == 0x5A)					 //应答距离
		{
			
			if(i != 0)
			{
				memcpy(packet_bytes, packet_bytes + 58 - i, 58 - i);
				serial_->read(packet_bytes + 58 - i, i);
			}
			if(packet_bytes[i+57] != CalCRC8(packet_bytes, 57))		break;
			int s = packet_bytes[i + 5];			
			s < 0 ? s += 256 : s;
			int z = packet_bytes[i + 6];
			z < 0 ? z += 256 : z;
			
				degree = (s * 256 + z) / 100.f;
			
			int s_e = packet_bytes[i + 55];
			s_e < 0 ? s_e += 256 : s_e;
			int z_e = packet_bytes[i + 56];
			z_e < 0 ? z_e += 256 : z_e;
			
			end_degree = (s_e * 256 + z_e) / 100.f;
			if(degree > end_degree)
				degree_interval = end_degree + 360 - degree;
			else
				degree_interval = end_degree - degree;	
			int invalidValue = 0;
		
			for (size_t num = 0; num < 48; num+=3)
			{
				int s = packet_bytes[i + num + 7];
				s < 0 ? s += 256 : s;
				int z = packet_bytes[i + num + 8];
				z < 0 ? z += 256 : z;
				int y_i = packet_bytes[i + num + 9];
				y_i < 0 ? y_i += 256 : y_i;
				if ((s * 256 + z) != 0xFFFF)
				{	
					if(idx<=600)
					{
					scan_points_[idx].range = double(s * 256 + (z)) / 1000.f;
					scan_points_[idx].intensity = int(y_i);
					idx++;
					count_sum++;
					}
				}
				else
				{
					invalidValue++;
				}
			}

			invalidValue = 16 - invalidValue;


			for (size_t i = 0; i < invalidValue; i++)
			{
				if(idx<=600)
				{
				if ((degree + (degree_interval / invalidValue * i)) > 360)
					scan_points_[idx-invalidValue+i].degree = degree + (degree_interval / invalidValue * i) - 360; 
				else
					scan_points_[idx-invalidValue+i].degree = degree + (degree_interval / invalidValue * i);
				}
			}

			if (degree < last_degree) 	
			{			
				idx = 0;
				
				//printf("ros2-before=%f\n ", scan_points_[k].range);

				for(int k=0;k<scan_points_.size();k++)
				{	
					if(angle_able_max > 360)
					{	
						if((360-scan_points_[k].degree) > (angle_able_max - 360) && (360-scan_points_[k].degree) < angle_able_min)
							{scan_points_[k].range = 0;  printf("angle_able_max > 360\n ");}

					}
					else 
					{
						if((360-scan_points_[k].degree) > angle_able_max || (360-scan_points_[k].degree) < angle_able_min)
							{scan_points_[k].range = 0; printf("else\n ");}
					}
					
					if(scan_points_[k].range < min_range || scan_points_[k].range > max_range)
						{//scan_points_[k].range = 0; 
							//printf("scan_points_[k].range=%f\n. min_range=%f\n. max_range=%f\n ",scan_points_[k].range,min_range,max_range);
							}
				}
				
				//printf("ros2-after=%f\n ", scan_points_[k].range);
				boost::unique_lock<boost::mutex> lock(mutex_);
				scan_points_bak_.resize(scan_points_.size());
				scan_points_bak_.assign(scan_points_.begin(), scan_points_.end());
				
				
 				for(int k=0; k<scan_points_.size(); k++)
				{
					scan_points_[k].range = 0;
					scan_points_[k].degree = 0;
				}
				pre_time_ = time_;
				lock.unlock();
				pubscan_cond_.notify_one();				
				time_ = rclcpp::Node::now();
			}

			last_degree = degree;
		}
	}		
  }	
  if (packet_bytes)
  {
    packet_bytes = NULL;
    delete packet_bytes;
  }

}

uint8_t LSN10::CalCRC8(unsigned char * p, int len)
{
  uint8_t crc = 0;
  int sum = 0;

  for (int i = 0; i < len; i++)
  {
    sum += uint8_t(p[i]);
  }
  crc = sum & 0xff;
  return crc;
}


void LSN10::pubScanThread()
{
  bool wait_for_wake = true;
  boost::unique_lock<boost::mutex> lock(pubscan_mutex_);

  while (rclcpp::ok() && !is_shutdown_)
  {
    while (wait_for_wake)
    {
      pubscan_cond_.wait(lock);
      wait_for_wake = false;
    }
    std::vector<ScanPoint> points;
    rclcpp::Time start_time;
    float scan_time;

    this->getScan(points, start_time, scan_time);
    int count = count_sum;//points.size();
    if (count <= 0)
      continue;
	int scan_num = (angle_able_max-angle_able_min)/360*count;

    sensor_msgs::msg::LaserScan msg;
    msg.header.frame_id = "laser";
    msg.header.stamp = start_time;
	count_sum = 0;
	if(angle_able_max >360)
	{
	msg.angle_min = 2 * M_PI * (angle_able_min-360) / 360;
    msg.angle_max = 2 * M_PI * (angle_able_max-360) / 360;
	}
	else 
	{
    msg.angle_min = 2 * M_PI * angle_able_min / 360;
    msg.angle_max = 2 * M_PI * angle_able_max / 360;
	}

    msg.angle_increment = 2 * M_PI / count;
    msg.range_min = 0;
    msg.range_max = 100;

    msg.ranges.resize(scan_num);
    msg.intensities.resize(scan_num);


    msg.scan_time = scan_time;
    msg.time_increment = scan_time / (double)(count - 1);
	
	for(int k=0; k<scan_num; k++)
	{
	msg.ranges[k] = std::numeric_limits<float>::infinity();
        msg.intensities[k] = 0;
	}
	int start_num = floor(angle_able_min * count/360);
	int end_num = floor(angle_able_max * count/360);
	for (int i = 0; i < count; i++) {
		int point_idx = (360-points[i].degree) * count/360;
		//printf("points[%d].degree = %lf\n",i,points[i].degree);
		//printf("i=%d point_idx=%d\n",i,point_idx);
		if(point_idx<(end_num-count))
			point_idx += count;
		point_idx =  point_idx - start_num;
 
		if(point_idx < 0 || point_idx > scan_num) 
			continue;
		if (points[i].range == 0.0) {
			msg.ranges[point_idx] = std::numeric_limits<float>::infinity();
			msg.intensities[point_idx] = 0;
		}
		else {
			double dist = points[i].range;
			msg.ranges[point_idx] = (float) dist;
			msg.intensities[point_idx] = points[i].intensity;
		}           
    }
    //printf("degree = %lf\n.range = %f\n.intensity = %d\n",points[300].degree,points[300].range,points[300].intensity);
    pub_->publish(msg);
    wait_for_wake = true;
  }
}
}

void handleSig(int signo)
{
  printf("handleSig\n");
  rclcpp::shutdown();
  exit(0);
}

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  signal(SIGINT, handleSig);
  signal(SIGTERM, handleSig);
  auto node = rclcpp::Node::make_shared("lidar10");
  ls::LSN10* lsn10 = ls::LSN10::instance();
  usleep(100000);
  rclcpp::spin(node);
  
  return 0;
}
