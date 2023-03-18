/*******************************************************
@company: Copyright (C) 2021, Leishen Intelligent System
@product: LSN10
@filename: lsn10.cpp
@brief:
@version:       date:       author:     comments:
@v1.0           21-8-21     leo          new
*******************************************************/
#include "lsn10/lsn10.h"
#include <stdio.h>
#include <signal.h> 

namespace ls{
LSN10 * LSN10::instance()
{
  static LSN10 obj;
  return &obj;
}

LSN10::LSN10()
{
  int code = 0;
  initParam();
  pub_ = n_.advertise<sensor_msgs::LaserScan>(scan_topic_, 3);
  serial_ = LSIOSR::instance(serial_port_, baud_rate_);
  code = serial_->init();
  if(code != 0)
  {
	printf("open_port %s ERROR !\n", serial_port_.c_str());
	ros::shutdown();
	exit(0);
  }
  printf("open_port %s  OK !\n", serial_port_.c_str());
  recv_thread_ = new boost::thread(boost::bind(&LSN10::recvThread, this));
  pubscan_thread_ = new boost::thread(boost::bind(&LSN10::pubScanThread, this));
  difop_switch = n_.subscribe<std_msgs::Int8>("rev_order", 1 ,&LSN10::rev_order,this);
}

LSN10::~LSN10()
{

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
}

void LSN10::rev_order(const std_msgs::Int8 msg){
  int i = msg.data;
  char data[188] = {0x00};
  if(i <= 1)
  {
  data[0] = 0xA5;
  data[1] = 0x5A;
  data[2] = 0x01;
  data[181] = char(i);
  data[184] = 0x06;
  data[185] = 0x01;
  data[186] = 0xFA;
  data[187] = 0xFB;
  }
  else if(i>=6 && i<=12){
  data[0] = 0xA5;
  data[1] = 0x5A;
  data[2] = 0x01;
  data[172] = char(i);
  data[184] = 0x0a;
  data[185] = 0x01;
  data[186] = 0xFA;
  data[187] = 0xFB;
  }
  else return;
  int rtn = serial_->send((const char*)data, 188);
  if (rtn < 0)
	printf("start scan error !\n");
  return ;
}

void LSN10::initParam()
{
  std::string scan_topic = "/scan";
  std::string frame_id = "laser_link";
  std::string port = "/dev/ttyUSB0";
  ros::NodeHandle nh("~");
  nh.param("versions", versions, 1);
  nh.param("scan_topic", scan_topic_, scan_topic);
  nh.param("frame_id", frame_id_, frame_id);
  nh.param("serial_port", serial_port_, port);
  nh.param("min_range", min_range, 0.3);
  nh.param("max_range", max_range, 100.0);
  nh.param("angle_disable_min", angle_disable_min,0.0);
  nh.param("angle_disable_max", angle_disable_max,0.0);
  nh.param("truncated_mode", truncated_mode_,0);
  nh.param<std::vector<int>>("disable_min", disable_angle_min_range, disable_angle_range_default);
  nh.param<std::vector<int>>("disable_max", disable_angle_max_range, disable_angle_range_default);
  count_sum = 0;
  while(angle_disable_min<0)	angle_disable_min+=360;
  while(angle_disable_max<0)	angle_disable_max+=360;
  while(angle_disable_min>360)	angle_disable_min-=360;
  while(angle_disable_max>360)	angle_disable_max-=360;
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
  points_size_ = 2000;
  scan_points_.resize(points_size_);
  if (versions == 1){
	PACKET_SIZE = 58;
	package_points = 16;
	data_bits_start = 7;
	degree_bits_start = 5;
	end_degree_bits_start = 55;
	baud_rate_= 230400;
	point_len = 3;
  } 
  else if (versions == 2){
	PACKET_SIZE = 108;
	package_points = 16;
	data_bits_start = 7;
	degree_bits_start = 5;
	end_degree_bits_start = 105;
	baud_rate_= 460800;
	point_len = 6;

  }
}

int LSN10::getScan(std::vector<ScanPoint> &points, ros::Time &scan_time, float &scan_duration)
{
  boost::unique_lock<boost::mutex> lock(mutex_);
  points.assign(scan_points_bak_.begin(), scan_points_bak_.end());
  scan_time = pre_time_;
  scan_duration = (time_ - pre_time_).toSec();
  if(scan_duration == 0)
  {
	  time_ = ros::Time::now();
	  scan_duration = (time_ - pre_time_).toSec();
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
  
  while(!is_shutdown_&&ros::ok()){
	int count_2=0;
	int count = serial_->read(packet_bytes, 1);
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
			ROS_WARN("serial open fail");
			usleep(300000);
		}
		link_time = 0;
	}
	if(count <= 0) 				continue;
	if(packet_bytes[0] != 0xA5)			continue;
	
	while(count_2 <= 0)
	{
		count_2 = serial_->read(packet_bytes+count, 1);
		count += count_2;
	}
	count_2 = 0;
	if(packet_bytes[1] != 0x5A)			continue;
	while(count < PACKET_SIZE)
	{
		count_2 = serial_->read(packet_bytes+count, PACKET_SIZE-count);
		count += count_2;
	}
	
	if(packet_bytes[PACKET_SIZE-1] != CalCRC8(packet_bytes, PACKET_SIZE-1))						break;

	int s = packet_bytes[degree_bits_start];			
	int z = packet_bytes[degree_bits_start+1];
	degree = (s * 256 + z) / 100.f;
			
	int s_e = packet_bytes[end_degree_bits_start];
	int z_e = packet_bytes[end_degree_bits_start+1];
	end_degree = (s_e * 256 + z_e) / 100.f;

	if(degree > end_degree)
		degree_interval = end_degree + 360 - degree;
	else
		degree_interval = end_degree - degree;	
	int invalidValue = 0;
	for (size_t num = 0; num < package_points*point_len; num+=point_len)
	{
		int s = packet_bytes[num + data_bits_start];
		int z = packet_bytes[num + data_bits_start+1];
		int y = packet_bytes[num + data_bits_start+2];
		if ((s * 256 + z) != 0xFFFF)
		{	
			if(idx<=1000)
			{
			scan_points_[idx].range = double(s * 256 + (z)) / 1000.f;
			scan_points_[idx].intensity = int(y);
			idx++;
			}
		}
		else
		{
			invalidValue++;
		}
		if(versions == 2)
		{
			s = packet_bytes[num + data_bits_start+3];
			z = packet_bytes[num + data_bits_start+4];
			y = packet_bytes[num + data_bits_start+5];
			scan_points_[idx+1000].range = double(s * 256 + (z)) / 1000.f;
			scan_points_[idx+1000].intensity = int(y);
		}
		
	}
		invalidValue = 16 - invalidValue;
		for (size_t i = 0; i < invalidValue; i++)
		{
			if(idx<=1000)
			{
			if ((degree + (degree_interval / (invalidValue-1) * i)) > 360)
				scan_points_[idx-invalidValue+i].degree = degree + (degree_interval / (invalidValue-1) * i) - 360; 
			else
				scan_points_[idx-invalidValue+i].degree = degree + (degree_interval / (invalidValue-1) * i);
			}
			
		}

		if (degree < last_degree) 	
		{			
			
			count_sum = idx;
			idx = 0;

			for(int k=0;k<1000;k++)
			{	

				if(angle_able_max > 360)
				{	
					if((360-scan_points_[k].degree) > (angle_able_max - 360) && (360-scan_points_[k].degree) < angle_able_min)
					{
						scan_points_[k].range = 0;
						scan_points_[k+1000].range = 0;
					}
				}
				else 
				{
					if((360-scan_points_[k].degree) > angle_able_max || (360-scan_points_[k].degree) < angle_able_min)
					{
						scan_points_[k].range = 0;
						scan_points_[k+1000].range = 0;
					}
				}
				
				if(scan_points_[k].range < min_range || scan_points_[k].range > max_range)
					scan_points_[k].range = 0;
				if(scan_points_[k+1000].range < min_range || scan_points_[k+1000].range > max_range)
					scan_points_[k+1000].range = 0;
			}
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
			time_ = ros::Time::now();
		}

		last_degree = degree;
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

  while (ros::ok() && !is_shutdown_)
  {
    while (wait_for_wake)
    {
      pubscan_cond_.wait(lock);
      wait_for_wake = false;
    }
		
    std::vector<ScanPoint> points;
    ros::Time start_time;
    float scan_time;
    this->getScan(points, start_time, scan_time);
    int count = count_sum;//points.size();
	int count_2 = count;
	count_sum = 0;
	if(versions == 2)		count_2 += count;
    if(count <= 0)      	continue;
    sensor_msgs::LaserScan msg;
    msg.header.frame_id = frame_id_;
    msg.header.stamp = start_time;
    msg.angle_min = -M_PI;
    msg.angle_max =  M_PI;
	msg.angle_increment = 2 * M_PI / (double)(count - 1);
    msg.range_min = min_range;
    msg.range_max = max_range;
    msg.ranges.resize(count_2);
    msg.intensities.resize(count_2);
    msg.scan_time = scan_time;
    msg.time_increment = scan_time / (double)(count - 1);
	
	for(int k=0; k<count_2; k++)
	{
		msg.ranges[k] = std::numeric_limits<float>::infinity();
        msg.intensities[k] = 0;
	}
	for (int i = 0; i < count; i++) {
		int point_idx = (360-points[i].degree) * count/360;
		if (points[i].range == 0.0) {
			msg.ranges[point_idx] = std::numeric_limits<float>::infinity();
			msg.intensities[point_idx] = 0;
		}
		else {
			double dist = points[i].range;
			msg.ranges[point_idx] = (float) dist;
			msg.intensities[point_idx] = points[i].intensity;
		}
		if(versions ==2)
		{
			if (points[i+1000].range == 0.0) {
				msg.ranges[point_idx+count] = std::numeric_limits<float>::infinity();
				msg.intensities[point_idx+count] = 0;
			}
			else {
				double dist = points[i+1000].range;
				msg.ranges[point_idx+count] = (float) dist;
				msg.intensities[point_idx+count] = points[i+1000].intensity;
			}
		}
		if(truncated_mode_==1)
		{
			for (int j = 0; j < disable_angle_max_range.size(); ++j) 
			{
		        if ((point_idx >= (disable_angle_min_range[j] * count / 360) ) &&
		              (point_idx <= (disable_angle_max_range[j] * count / 360 ))) 
			  {
		            msg.ranges[point_idx] = std::numeric_limits<float>::infinity();
		            msg.intensities[point_idx] = 0;
		          }
		        }
		}
		if(msg.intensities[point_idx] <=5 && msg.intensities[point_idx] >0)
		{
			msg.ranges[point_idx] = std::numeric_limits<float>::infinity();
			msg.intensities[point_idx] =0;
		}
    }
    pub_.publish(msg);
    wait_for_wake = true;
  }
}
}

void handleSig(int signo)
{
  printf("handleSig\n");
  ros::shutdown();
  exit(0);
}

int main(int argv, char **argc)
{
  signal(SIGINT, handleSig);
  signal(SIGTERM, handleSig);
  ros::init(argv, argc, "lsn10");
 
  ls::LSN10* lsn10 = ls::LSN10::instance();
  usleep(100000);
  ros::spin();
  return 0;
}
