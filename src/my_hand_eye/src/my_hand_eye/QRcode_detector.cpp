#include <opencv2/opencv.hpp>
#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.hpp>
#include <my_hand_eye/ArrayofTaskArrays.h>

#include "my_hand_eye/QRcode_detector.h"

PLUGINLIB_EXPORT_CLASS(my_hand_eye::QRcodeDetector, nodelet::Nodelet);

namespace my_hand_eye
{
	void QRcodeDetector::onInit()
	{
		nh_ = getMTNodeHandle();
		pnh_ = getMTPrivateNodeHandle();
		QR_code_subscriber_ = nh_.subscribe<std_msgs::String>("/barcode", 10, &QRcodeDetector::QRcodeCallback, this);
		QR_code_publisher_ =
			nh_.advertise<my_hand_eye::ArrayofTaskArrays>("/task", 10,
														  boost::bind(&QRcodeDetector::connectCallback, this),
														  boost::bind(&QRcodeDetector::disconnectCallback, this));
		esp32_timer_ = nh_.createTimer(ros::Rate(10), &QRcodeDetector::esp32Callback, this, false, false);
		flag_ = false;
		std::string usart_port_name = pnh_.param<std::string>("usart_port_name", "/dev/esp32"); // Fixed serial port number //固定串口号
		int serial_baud_rate = pnh_.param<int>("serial_baud_rate", 9600);						// Communicate baud rate 115200 to the lower machine //和下位机通信波特率9600
		try
		{
			esp32_serial_.setPort(usart_port_name);
			esp32_serial_.setBaudrate(serial_baud_rate);
			NODELET_INFO("baud rate: %d", serial_baud_rate);
			serial::Timeout _time = serial::Timeout::simpleTimeout(2000); // Timeout //超时等待
			esp32_serial_.setTimeout(_time);
			esp32_serial_.open();
		}
		catch (serial::IOException &e)
		{
			NODELET_ERROR("QRcodeDetector can not open serial port, Please check the serial port cable! "); // If opening the serial port fails, an error message is printed //如果开启串口失败，打印错误信息
		}
		if (esp32_serial_.isOpen())
		{
			esp32_timer_.start();
			NODELET_INFO("QRcodeDetector serial port opened.");
		}
	}

	void QRcodeDetector::QRcodeCallback(const std_msgs::StringConstPtr &info)
	{
		checkString(info->data);
		// NODELET_INFO("Succeeded to detect QR Code!");
	}

	void QRcodeDetector::connectCallback()
	{
		if (!QR_code_subscriber_ && QR_code_publisher_.getNumSubscribers() > 0)
		{
			NODELET_INFO("Connecting to barcode topic.");
			QR_code_subscriber_ = nh_.subscribe<std_msgs::String>("/barcode", 10, &QRcodeDetector::QRcodeCallback, this);
		}
	}

	void QRcodeDetector::disconnectCallback()
	{
		if (QR_code_publisher_.getNumSubscribers() == 0)
		{
			NODELET_INFO("Unsubscribing from barcode topic.");
			QR_code_subscriber_.shutdown();
			esp32_serial_.close();
		}
	}

	void QRcodeDetector::esp32Callback(const ros::TimerEvent &event)
	{										   // Intermediate variable //中间变量
		uint8_t Receive_Data_Pr[1]; // Temporary variable to save the data of the lower machine //临时变量，保存下位机数据										 // Static variable for counting //静态变量，用于计数
		std::string str;
		const int DATA_SIZE = 10;
		str.reserve(DATA_SIZE);
		while (true)
		{
			std::size_t num = esp32_serial_.read(Receive_Data_Pr, sizeof(Receive_Data_Pr));
			if (Receive_Data_Pr[0] == 0X00 || num != sizeof(Receive_Data_Pr))
			{
				break;
			}
			if (Receive_Data_Pr[0] == FRAME_HEADER || str.size()) // Ensure that the first data in the array is FRAME_HEADER //确保数组第一个数据为FRAME_HEADER
			{
				str.push_back(Receive_Data_Pr[0]);
			}
			else
			{
				continue;
			}
			if (str.size() >= DATA_SIZE)
			{
				if (str[DATA_SIZE - 1] == FRAME_TAIL)
				{
					std::string ss = str.substr(0, DATA_SIZE - 2);
					uint8_t check = 0;
					for (const uint8_t &ch : ss)
					{
						check = check ^ ch;
					}
					if (check == str[DATA_SIZE - 2])
					{
						// 去掉帧头
						ss.erase(ss.begin());
						break;
					}
				}
				str.clear();
			}
		}
	}

	bool QRcodeDetector::checkString(const std::string &str)
	{
		my_hand_eye::ArrayofTaskArrays arr;
		arr.loop.reserve(2);
		if (str.size() != 7)
		{
			NODELET_WARN("String has invalid size %ld.", str.size());
			return false;
		}
		cv::Mat resImg;
		if (!flag_)
			resImg = cv::Mat(cv::Size(1920, 1080), CV_8UC3, cv::Scalar(100, 80, 60));
		std::string num;
		my_hand_eye::TaskArray tarr;
		uint8_t task;
		bool note[4] = {false};
		int ii = 0;
		for (size_t i = 0; i < str.size(); i++)
		{
			if (i != 3) // 不显示 +
			{
				char txt_temp[2] = {str[i]};
				if (!flag_)
					putText(resImg, txt_temp, cv::Point(ptx_info[ii] + txt_Xoffset, pty_info[ii] + txt_Yoffset),
							cv::FONT_HERSHEY_PLAIN, txt_size, cv::Scalar::all(255), txt_thick, cv::FILLED, false);
				if (str[i] < '1' || str[i] > '3')
				{
					NODELET_WARN("String has invalid characters %c.", str[i]);
					return false;
				}
				task = str[i] - '0';
				if (note[task])
				{
					NODELET_WARN("String has duplicate characters %c.", str[i]);
					return false;
				}
				note[task] = true;
				tarr.task[ii % 3] = task;
				ii++;
			}
			else
			{
				if (str[i] != '+')
				{
					NODELET_WARN("String miss the '+'");
					return false;
				}
				arr.loop.push_back(tarr);
				memset(note, 0, sizeof(note));
			}
		}
		arr.loop.push_back(tarr);
		QR_code_publisher_.publish(arr);
		uint8_t data[1] = {true};
		try
		{
			esp32_serial_.write(data, sizeof(data)); // Send data to the serial port //向串口发数据
		}
		catch (serial::IOException &e)
		{
			NODELET_ERROR("Unable to send data through serial port"); // If sending data fails, an error message is printed //如果发送数据失败,打印错误信息
		}
		screenShow(resImg);
		return true;
	}

	void QRcodeDetector::screenShow(cv::Mat &img)
	{
		if (flag_)
			return;
		cv::namedWindow("resImg", cv::WINDOW_NORMAL);
		cv::setWindowProperty("resImg", cv::WND_PROP_FULLSCREEN, cv::WINDOW_FULLSCREEN);
		cv::resize(img, img, cv::Size(480, 640));
		imshow("resImg", img);
		cv::waitKey(500);
		boost::lock_guard<boost::mutex> lk(mtx_);
		flag_ = true;
	}
}
