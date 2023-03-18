#include "my_hand_eye/arm_controller.h"

using namespace cv;
using namespace std;

my_hand_eye::ArmController ac;
Mat srcImg;
int num = 0;
const int maxnum = 30;
char text[100] = {};
ofstream File;
ofstream Matrix;
//制定运动位置
double targetx = -0.563389;
double targety = 24.7766;
double targetz = 13.6579;
//实现双击图片，进行截图的函数
void MouseEvent(int event, int x, int y, int flags, void* data)
{
	if (event == EVENT_LBUTTONDBLCLK)
	{
		num++;		
		//imshow("cut", srcImg);
		snprintf(text, sizeof(text), "/home/fu/apriltag_ws/src/my_hand_eye/img/ApriltagRaw_%d.png", num);
		/*
			这里需要重新设置一下双引号里面图片的保存路径，ZZY_%d是图片名称
			%d是标识符，值即为num
			.jpg是文件拓展名，一般为jpg格式就行
		*/
		imwrite(text, srcImg);
		strcat(text,"\n");
		ROS_INFO("%s",text);
		if (File.is_open())
		{
			File << text;
		}
		ROS_INFO("第%d张图片保存成功\n", num);
	}
}

int main(int argc, char **argv)
{
    setlocale(LC_ALL, "");
	ac.ps_.go_to(targetx, targety, targetz, true, true);
	VideoCapture USBCamera;
	//设备编号根据实际而定，一般电脑摄像头默认是202
	USBCamera = VideoCapture(202);
	if (!USBCamera.isOpened())
		return -1;
	//需要在这里重新设置读取的视频宽高，opencv默认640*480
	USBCamera.set(CAP_PROP_FRAME_WIDTH, 1920);
	USBCamera.set(CAP_PROP_FRAME_HEIGHT, 1080);
	ROS_INFO_STREAM("\n视频的宽为：" << USBCamera.get(CAP_PROP_FRAME_WIDTH) << "\n视频的高为：" << USBCamera.get(CAP_PROP_FRAME_HEIGHT));
	File.open("/home/fu/apriltag_ws/src/my_hand_eye/img/img_info.txt", ios::binary|ios::out|ios::ate);
	namedWindow("Video");
	setMouseCallback("Video", MouseEvent, 0);
	while (num < maxnum)
	{
		USBCamera >> srcImg;
		if (!srcImg.data)
		{
			File.close();
			return -2;
		}
		if ((char)waitKey(1) == 27)
		{
			File.close();
			return -3;
		}
		imshow("Video", srcImg);
	}
	return 0;
}