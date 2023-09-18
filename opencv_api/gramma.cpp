#include<opencv.hpp>
#include<iostream>

using namespace cv;
using namespace std;

//gramma参数，暗部增强取0-99，亮部增强取101-300。取100为原图
int Factor = 40;

void grammaTransForm(int Factor, Mat &resImg);
void Saturation_2(cv::Mat &src, int percent);

#define max2(a,b) (a>b?a:b)
#define max3(a,b,c) (a>b?max2(a,c):max2(b,c))
#define min2(a,b) (a<b?a:b)
#define min3(a,b,c) (a<b?min2(a,c):min2(b,c))

int main()
{
	Mat srcImg = imread("E:\\CodeRepositories\\opencv_test\\car\\明暗3.jpg");
	if (!srcImg.data)
		return -1;
	imshow("srcImg", srcImg);

	//原始图像H分量
	Mat HSV_Img;
	cvtColor(srcImg, HSV_Img, COLOR_BGR2HSV);
	vector<Mat> HSV_plane;
	split(HSV_Img, HSV_plane);
	cv::imshow("H_Origin", HSV_plane[0]);

	//*******
	// 以下为使用roi区域有选择的调节暗部区域
	//*******
	
	//转换为灰度图，寻找高光区域
	Mat Gray;
	cvtColor(srcImg, Gray, COLOR_BGR2GRAY);
	//阈值化分离高光区域
	Mat Thresh_Gray;
	threshold(Gray, Thresh_Gray, 120, 255, THRESH_BINARY);
	//取反获得暗部区域
	bitwise_not(Thresh_Gray, Thresh_Gray);
	cv::imshow("Thresh_origin", Thresh_Gray);

	//膨胀操作，消除小噪点，也可以不用这一步，因为后面也会根据轮廓大小进行筛除小噪点
	Mat kernel = getStructuringElement(MORPH_RECT, Size(3, 3), Point(-1, -1));
	dilate(Thresh_Gray, Thresh_Gray, kernel);
	cv::imshow("Thresh_dilate", Thresh_Gray);

	//轮廓查找
	vector<vector<Point>> contours;
	vector<Vec4i> hierarchy;
	findContours(Thresh_Gray, contours, hierarchy, RETR_EXTERNAL, CHAIN_APPROX_NONE);

	//按区域gramma矫正
	Mat Img_clone = srcImg.clone();
	for (size_t i = 0; i < contours.size(); i++)
	{
		//滤除太小的区域
		if (contourArea(contours[i]) < 10000) continue;
		//创建掩膜
		Rect roi = boundingRect(contours[i]);
		Mat mask = Img_clone(roi);
		//gramma矫正，并对每个区域进行颜色增强
		grammaTransForm(Factor, mask);
		Saturation_2(mask, 40);
	}
	//全局颜色增强
	Saturation_2(Img_clone, 10);
	imshow("resImg", Img_clone);

	//处理后图像H分量
	Mat HSV_Res_Img;
	cvtColor(Img_clone, HSV_Res_Img, COLOR_BGR2HSV);
	vector<Mat> HSV_Res_plane;
	split(HSV_Res_Img, HSV_Res_plane);
	cv::imshow("H_Res", HSV_Res_plane[0]);

	// *******
	// 也可以对整张图片进行全局gramma矫正，直接使用grammaTransForm(int Factor, Mat &resImg)即可
	// *******

	waitKey(0);
	return 0;
}

void grammaTransForm(int Factor, Mat &FImg)
{
	float kFactor = (float)Factor / 100;
	unsigned char LUT[256];
	for (size_t i = 0; i < 256; i++)
	{
		LUT[i] = saturate_cast<uchar>(pow((float)i / 255.0, kFactor) * 255.0f);
	}
	if (FImg.channels() == 1)
	{
		MatIterator_<uchar> iterator = FImg.begin<uchar>();
		MatIterator_<uchar> iteratorEnd = FImg.end<uchar>();
		for (; iterator != iteratorEnd; iterator++)
			*iterator = LUT[(*iterator)];
	}
	else
	{
		MatIterator_<Vec3b> iterator = FImg.begin<Vec3b>();
		MatIterator_<Vec3b> iteratorEnd = FImg.end<Vec3b>();
		for (; iterator != iteratorEnd; iterator++)
		{
			(*iterator)[0] = LUT[((*iterator)[0])];
			(*iterator)[1] = LUT[((*iterator)[1])];
			(*iterator)[2] = LUT[((*iterator)[2])];
		}
	}
}

void Saturation_2(cv::Mat &src, int percent)
{
	float Increment = percent * 1.0f / 100;
	cv::Mat temp = src.clone();
	int row = src.rows;
	int col = src.cols;
	for (int i = 0; i < row; ++i)
	{
		uchar* s = temp.ptr<uchar>(i);
		uchar* t = src.ptr<uchar>(i);
		for (int j = 0; j < col; ++j)
		{
			uchar b = s[3 * j];
			uchar g = s[3 * j + 1];
			uchar r = s[3 * j + 2];
			float max = max3(r, g, b);
			float min = min3(r, g, b);
			float delta, value;
			float L, S, alpha;
			delta = (max - min) / 255;
			if (delta == 0)
				continue;
			value = (max + min) / 255;
			L = value / 2;
			if (L < 0.5)
				S = delta / value;
			else
				S = delta / (2 - value);
			if (Increment >= 0)
			{
				if ((Increment + S) >= 1)
					alpha = S;
				else
					alpha = 1 - Increment;
				alpha = 1 / alpha - 1;
				t[3 * j + 2] = static_cast<uchar>(r + (r - L * 255) * alpha);
				t[3 * j + 1] = static_cast<uchar>(g + (g - L * 255) * alpha);
				t[3 * j] = static_cast<uchar>(b + (b - L * 255) * alpha);
			}
			else
			{
				alpha = Increment;
				t[3 * j + 2] = static_cast<uchar>(L * 255 + (r - L * 255) * (1 + alpha));
				t[3 * j + 1] = static_cast<uchar>(L * 255 + (g - L * 255) * (1 + alpha));
				t[3 * j] = static_cast<uchar>(L * 255 + (b - L * 255) * (1 + alpha));
			}
		}
	}
}